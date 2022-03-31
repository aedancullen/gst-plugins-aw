/* GStreamer sunxifbsink plugin
 * Copyright (C) 2013 Harm Hanemaaijer <fgenfb@yahoo.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */
/**
 * SECTION:element-sunxifbsink
 *
 * The sunxifbsink element intends implements a hardware accelerated
 * video sink for the console framebuffer of Allwinner A1x/A20-based
 * devices. The basis of the implementation is the optimized fbdev
 * sink as implemented in the GstFbdevFramebufferSink class.
 *
 * <refsect2>
 * <title>Property settings,<title>
 * <para>
 * The plugin comes with variety of configurable properties regulating
 * the size and frames per second of the video output, and various
 * options regulating the rendering method (including rendering directly
 * to video memory and page flipping).
 * </para>
 * </refsect2>
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v videotestsrc ! sunxifbsink >/dev/null
 * ]|
 * Output the video test signal to the framebuffer. The redirect to
 * null surpressed interference from console text mode.
 * |[
 * gst-launch -v videotestsrc ! sunxifbsink full-screen=true
 * ]|
 * Run videotstsrc at full-screen resolution
 * |[
 * gst-launch -v videotestsrc horizontal_speed=10 ! sunxifbsink \
 * full-screen=true buffer-pool=true graphics-mode=true
 * ]|
 * This command illustrates some of the plugin's optimization features
 * by rendering to video memory with vsync and page flipping in
 * console graphics mode. There should be no tearing with page flipping/
 * vsync enabled. You might have to use the fps property to reduce the frame
 * rate on slower systems.
 * |[
 * gst-launch playbin uri=[uri] video-sink="sunxifbsink full-screen=true"
 * ]|
 * Use playbin while passing options to sunxifbsink.
 * </refsect2>
 * <refsect2>
 * <title>Caveats</title>
 * <para>
 * The actual implementation of the Linux framebuffer API varies between
 * systems, and methods beyond the most basic operating mode may not work
 * correctly on some systems. This primarily applies to page flipping
 * and vsync. The API implementation may be slower than expected on certain
 * hardware due to, for example, extra hidden vsyncs being performed in the
 * pan function. The "pan-does-vsync" option may help in that case.
 * </para>
 * </refsect2>
 */

/*
 * The hardware addressing portion of the plugin was adapted from
 * xf86-video-sunxifb, which has the following copyright message;
 *
 * Copyright © 2013 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <linux/kd.h>
#include <glib/gprintf.h>

#include <gst/gst.h>
#include <gst/video/gstvideosink.h>
#include <gst/video/video.h>
#include <gst/video/video-info.h>
#include "gstsunxifbsink.h"
#include <ion_mem_alloc.h>
#include "sunxi_tr.h"
#include "g2d_driver_enh.h"

GST_DEBUG_CATEGORY_STATIC (gst_sunxifbsink_debug_category);
#define GST_CAT_DEFAULT gst_sunxifbsink_debug_category

static inline void GST_SUNXIFBSINK_ERROR_OBJECT (GstSunxifbsink * sunxifbsink,
const gchar *message) {
  if (!sunxifbsink->fbdevframebuffersink.framebuffersink.silent)
    g_print ("\033[;31m%s\n\033[0m", message);
  else
    GST_ERROR_OBJECT (sunxifbsink,"%"GST_PTR_FORMAT, message);
}

static inline void GST_SUNXIFBSINK_MESSAGE_OBJECT (GstSunxifbsink * sunxifbsink,
const gchar *message) {
  if (!sunxifbsink->fbdevframebuffersink.framebuffersink.silent)
    g_print ("%s.\n", message);
  else
    GST_INFO_OBJECT (sunxifbsink,"%"GST_PTR_FORMAT, message);
}

#define ALIGNMENT_GET_ALIGN_BYTES(offset, align) \
    (((align) + 1 - ((offset) & (align))) & (align))
#define ALIGNMENT_GET_ALIGNED(offset, align) \
    ((offset) + ALIGNMENT_GET_ALIGN_BYTES(offset, align))
#define ALIGNMENT_APPLY(offset, align) \
    offset = ALIGNMENT_GET_ALIGNED(offset, align);

/* Class function prototypes. */
static gboolean gst_sunxifbsink_open_hardware (
    GstFramebufferSink *framebuffersink, GstVideoInfo *info,
    gsize *video_memory_size, gsize *pannable_video_memory_size);
static void gst_sunxifbsink_close_hardware (
    GstFramebufferSink *framebuffersink);
static GstVideoFormat *gst_sunxifbsink_get_supported_overlay_formats (
    GstFramebufferSink *framebuffersink);
static gboolean gst_sunxifbsink_get_overlay_video_alignment (
    GstFramebufferSink *framebuffersink, GstVideoInfo *video_info,
    GstFramebufferSinkOverlayVideoAlignment *video_alignment,
    gint *overlay_align, gboolean *video_alignment_matches);
static gboolean gst_sunxifbsink_prepare_overlay (
    GstFramebufferSink *framebuffersink, GstVideoFormat format);
static GstFlowReturn gst_sunxifbsink_show_overlay (
    GstFramebufferSink *framebuffersink, GstMemory *memory);

static gboolean gst_sunxifbsink_reserve_layer (GstSunxifbsink *sunxifbsink);
static void gst_sunxifbsink_release_layer (GstSunxifbsink *sunxifbsink);
static gboolean gst_sunxifbsink_show_layer (GstSunxifbsink *sunxifbsink);
static void gst_sunxifbsink_hide_layer (GstSunxifbsink *sunxifbsink);

enum
{
  PROP_0,
};

#define GST_SUNXIFBSINK_TEMPLATE_CAPS \
        GST_VIDEO_CAPS_MAKE ("RGB") \
        "; " GST_VIDEO_CAPS_MAKE ("BGR") \
        "; " GST_VIDEO_CAPS_MAKE ("RGBx") \
        "; " GST_VIDEO_CAPS_MAKE ("BGRx") \
        "; " GST_VIDEO_CAPS_MAKE ("xRGB") \
        "; " GST_VIDEO_CAPS_MAKE ("xBGR") \
        "; " GST_VIDEO_CAPS_MAKE ("NV12") \
        "; " GST_VIDEO_CAPS_MAKE ("NV21") \
        "; " GST_VIDEO_CAPS_MAKE ("YV12") \
        "; " GST_VIDEO_CAPS_MAKE ("I420") \
        "; " GST_VIDEO_CAPS_MAKE ("YUY2") \
        "; " GST_VIDEO_CAPS_MAKE ("UYVY") \
        "; " GST_VIDEO_CAPS_MAKE ("Y444") \
        "; " GST_VIDEO_CAPS_MAKE ("AYUV") ", " \
        "framerate = (fraction) [ 0, MAX ], " \
        "width = (int) [ 1, MAX ], " "height = (int) [ 1, MAX ]"

static GstStaticPadTemplate gst_sunxifbsink_sink_template =
GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_SUNXIFBSINK_TEMPLATE_CAPS)
    );

static GstVideoFormat sunxifbsink_supported_overlay_formats_table[] = {
  /* List the formats that support odds widths first. */
  GST_VIDEO_FORMAT_YV12,
  GST_VIDEO_FORMAT_I420,
  GST_VIDEO_FORMAT_NV12,
  GST_VIDEO_FORMAT_NV21,
  GST_VIDEO_FORMAT_AYUV,
  GST_VIDEO_FORMAT_BGRx,
  /* These formats do not properly support odd widths. */
  GST_VIDEO_FORMAT_YUY2,
  GST_VIDEO_FORMAT_UYVY,
  GST_VIDEO_FORMAT_Y444,
  GST_VIDEO_FORMAT_UNKNOWN
};

#define TRANSFORM_DEV_TIMEOUT 200
#define ALIGN_32B(x) (((x) + (31)) & ~(31))
#define ALIGN_16B(x) (((x) + (15)) & ~(15))

/* Class initialization. */

#define gst_sunxifbsink_parent_class fbdevframebuffersink_parent_class
G_DEFINE_TYPE_WITH_CODE (GstSunxifbsink, gst_sunxifbsink,
  GST_TYPE_FBDEVFRAMEBUFFERSINK,
  GST_DEBUG_CATEGORY_INIT (gst_sunxifbsink_debug_category, "sunxifbsink", 0,
  "debug category for sunxifbsink element"));

static void
gst_sunxifbsink_class_init (GstSunxifbsinkClass* klass)
{
/*  GObjectClass *gobject_class = G_OBJECT_CLASS (klass); */
  GstFramebufferSinkClass *framebuffer_sink_class =
      GST_FRAMEBUFFERSINK_CLASS (klass);

/*  gobject_class->set_property = gst_sunxifbsink_set_property;
    gobject_class->get_property = gst_sunxifbsink_get_property; */

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS(klass),
      gst_static_pad_template_get (&gst_sunxifbsink_sink_template));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS(klass),
      "Accelerated console framebuffer video sink for sunxi-based devices",
      "Sink/Video",
      "sunxi framebuffer sink",
      "Harm Hanemaaijer <fgenfb@yahoo.com>");

  framebuffer_sink_class->open_hardware =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_open_hardware);
  framebuffer_sink_class->close_hardware =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_close_hardware);
  framebuffer_sink_class->get_supported_overlay_formats =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_get_supported_overlay_formats);
  framebuffer_sink_class->get_overlay_video_alignment =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_get_overlay_video_alignment);
  framebuffer_sink_class->prepare_overlay =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_prepare_overlay);
  framebuffer_sink_class->show_overlay =
      GST_DEBUG_FUNCPTR (gst_sunxifbsink_show_overlay);
}

/* Class member functions. */

static void
gst_sunxifbsink_init (GstSunxifbsink *sunxifbsink) {
	GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->sunxifbsink init");
}

static gboolean
gst_sunxifbsink_open_hardware (GstFramebufferSink *framebuffersink,
    GstVideoInfo *info, gsize *video_memory_size,
    gsize *pannable_video_memory_size)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
  unsigned long arg[4] = {0};

  if(((access("/dev/zero",F_OK)) < 0)||((access("/dev/fb0",F_OK)) < 0)){
      printf("/dev/zero OR /dev/fb0 is not exit\n");
  }else{
      system("dd if=/dev/zero of=/dev/fb0");//clean the framebuffer
  }

  if (!gst_fbdevframebuffersink_open_hardware(framebuffersink, info,
      video_memory_size, pannable_video_memory_size))
    return FALSE;

  sunxifbsink->hardware_overlay_available = FALSE;

  if (framebuffersink->use_hardware_overlay == FALSE)
    return TRUE;

  sunxifbsink->fd_disp = open ("/dev/disp", O_RDWR);
  if (sunxifbsink->fd_disp < 0){
      GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink,
          "-->open /dev/disp error.");
    return TRUE;
  }

  sunxifbsink->fd_transform = open("/dev/transform",O_RDWR);
  if (sunxifbsink->fd_transform < 0){
        GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink,
            "-->/dev/transform does not exist, hardware rotation is not supported.");
    }

  if(sunxifbsink->fd_transform > 0){
    arg[0] = (unsigned long)&(sunxifbsink->transform_channel);
    if(ioctl(sunxifbsink->fd_transform, TR_REQUEST,(void*)arg)<0){
      GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->tr_request failed!");
      return -1;
    }

    //* set tr timeout
    arg[0] = sunxifbsink->transform_channel;
    arg[1] = TRANSFORM_DEV_TIMEOUT;
    if(ioctl(sunxifbsink->fd_transform, TR_SET_TIMEOUT,(void*)arg) != 0)
    {
      GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->tr_set_timeout failed!");
      return -1;
    }
  }

#ifdef __SUNXI_G2D_ROTATE__
  sunxifbsink->fd_g2d = open ("/dev/g2d", O_RDWR);
  if (sunxifbsink->fd_g2d < 0){
      GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink,
          "-->open /dev/g2d error.");
      return TRUE;
  }
#endif

  if (!gst_sunxifbsink_reserve_layer(sunxifbsink)) {
    GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink, "-->sunxifbsink reserver layer failed.");
    close(sunxifbsink->fd_disp);
    return TRUE;
  }

  sunxifbsink->layer_is_visible = FALSE;
  sunxifbsink->hardware_overlay_available = TRUE;
  GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->Hardware overlay available");

  sunxifbsink->sBuffer= g_new0(OmxPrivateBuffer, 1);

  return TRUE;
}

static void
gst_sunxifbsink_close_hardware (GstFramebufferSink *framebuffersink) {
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
  GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->sunxifbsink close");
  struct SunxiMemOpsS* ops =  GetMemAdapterOpsS();

  g_free(sunxifbsink->sBuffer);

  if (sunxifbsink->hardware_overlay_available) {
    gst_sunxifbsink_hide_layer(sunxifbsink);
    gst_sunxifbsink_release_layer(sunxifbsink);
  }
  /* Before calling close_hardware, use_hardware_overlay is expected to have
     been reset to the original value it had when open_hardware was called. */
  if (framebuffersink->use_hardware_overlay)
    close(sunxifbsink->fd_disp);

  if(sunxifbsink->rotate_addr_phy[0] != NULL)
  {
	SunxiMemPfree(ops,sunxifbsink->rotate_addr_phy[0]);
  }

  if(sunxifbsink->rotate_addr_phy[1] != NULL)
  {
	SunxiMemPfree(ops,sunxifbsink->rotate_addr_phy[1]);
  }
  gst_fbdevframebuffersink_close_hardware (framebuffersink);

  if(sunxifbsink->fd_transform >= 0)
	close(sunxifbsink->fd_transform);

#ifdef __SUNXI_G2D_ROTATE__
  if(sunxifbsink->fd_g2d >= 0)
	close(sunxifbsink->fd_g2d);
#endif
}

static GstVideoFormat *
gst_sunxifbsink_get_supported_overlay_formats (
    GstFramebufferSink *framebuffersink)
{

  return sunxifbsink_supported_overlay_formats_table;
}

/* Return the video alignment (top/bottom/left/right padding and stride
   alignment for each plane) that  is required to display the overlay
   described by video_info. Also returns the alignment requirement of the
   start address of the overlay in video memory. video_alignment_matches
   is set to TRUE if the alignment defined by video_info did not have to be
   adjusted, FALSE otherwise. The function returns TRUE if hardware overlay
   with given video info is supported, FALSE otherwise. */

gboolean
gst_sunxifbsink_get_overlay_video_alignment(GstFramebufferSink *framebuffersink,
    GstVideoInfo *video_info, GstFramebufferSinkOverlayVideoAlignment *
    video_alignment, gint *overlay_align, gboolean *video_alignment_matches)
{
  GstVideoFormat format;
  format = GST_VIDEO_INFO_FORMAT (video_info);
  if (format == GST_VIDEO_FORMAT_I420 ||
      format == GST_VIDEO_FORMAT_YV12 ||
      format == GST_VIDEO_FORMAT_NV12 ||
      format == GST_VIDEO_FORMAT_NV21) {
    if (GST_VIDEO_INFO_WIDTH (video_info) & 1)
      /* Hardware overlay not supported for odd widths for all planar formats
         except Y444. Although it almost works for odd widths, there is an
         artifact line at the right of the scaled area, related to the
         alignment requirements of the width. */
      return FALSE;
  }
  /* When uses other formats, some artifacts have been observed when the width
     is odd, but for now leave support for odd widths enabled. */
  *overlay_align = 15;
  /* For the Allwinner hardware overlay, scanlines need to be aligned to pixel
     boundaries with a minimum alignment of word-aligned. This is a good match
     for the buffer format generally provided by upstream, so direct video
     memory buffer pool streaming is almost always possible. */
  gst_framebuffersink_set_overlay_video_alignment_from_scanline_alignment (
      framebuffersink, video_info, 3, TRUE, video_alignment,
      video_alignment_matches);
  return TRUE;
}

/*
 * For the prepare overlay and show overlay functions, the parameters are
 * stored in the following fields:
 * framebuffersink->overlay_plane_offset[i] is the offset in bytes of each
 *     plane. Any top or left padding returned by get_overlay_video_alignment()
 *     will come first.
 * framebuffersink->overlay_scanline_offset[i] is the offset in bytes of the
 *     first pixel of each scanline for each plane (corresponding with the left
 *     padding * bytes per pixel). Usually 0.
 * framebuffersink->overlay_scanline_stride[i] is the scanline stride in bytes
 *     of each plane.
 * framebuffersink->videosink.width is the source width.
 * framebuffersink->videosink.height is the source height.
 * framebuffersink->video_rectangle.x is the destination x coordinate.
 * framebuffersink->video_rectangle.y is the destination y coordinate.
 * framebuffersink->video_rectangle.w is the destination width.
 * framebuffersink->video_rectangle.h is the destination height.
 */

static gboolean
gst_sunxifbsink_prepare_overlay (GstFramebufferSink *framebuffersink,
    GstVideoFormat format)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
  gchar s[256];

  g_sprintf(s,"---->sunxifb pre overlay(SCWxSCN=%d x %d,out_rec=[%d %d,%d,%d])",
	framebuffersink->videosink.width ,
	framebuffersink->videosink.height,
	framebuffersink->video_rectangle.x,
	framebuffersink->video_rectangle.y,
	framebuffersink->video_rectangle.w,
	framebuffersink->video_rectangle.h);
  GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, s);

  if (sunxifbsink->layer_is_visible)
    gst_sunxifbsink_hide_layer(sunxifbsink);

  sunxifbsink->overlay_format = format;

  return TRUE;
}

int hwRotateVideoPicture(GstSunxifbsink *sunxifbsink,tr_info *info)
{
    int ret = 0;
    unsigned long arg[4] = {0};
    tr_info tTrInfo;
	memcpy(&tTrInfo, info, sizeof(tr_info));
setup_tr:
    //* setup rotate
    arg[0] = sunxifbsink->transform_channel;
    arg[1] = (unsigned long)&tTrInfo;
    arg[2] = 0;
    arg[3] = 0;

    if(ioctl(sunxifbsink->fd_transform, TR_COMMIT,(void*)arg) != 0)
    {
        return -1;
    }

    //* wait
    arg[0] = sunxifbsink->transform_channel;
    arg[1] = 0;
    arg[2] = 0;
    arg[3] = 0;

    ret = ioctl(sunxifbsink->fd_transform,TR_QUERY,(void*)arg);

    while(1)// 0 : suceef; 1:busy ; -1:timeOut
    {
        ret = ioctl(sunxifbsink->fd_transform, TR_QUERY,(void*)arg);
		if(ret == 1)
		{
		usleep(1*1000);
		}
		else
			break;
    }

    //* if the tr is timeout ,we should setup tr again
    if(ret == -1)
    {
        goto setup_tr;
    }

    return ret;
}

static GstFlowReturn
gst_sunxifbsink_show_memory_yuv_planar (GstFramebufferSink *framebuffersink,
	GstVideoFormat format,GstMemory *mem)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
    luapi_layer_config luapiconfig;
	GstMapInfo mapinfo;
	char * phymem_start = 0;
	struct SunxiMemOpsS* ops =  GetMemAdapterOpsS();
	tr_info trans_info;
	static int m = 0;
	unsigned int width_align;
	unsigned int height_align;
	int rect_width;
	int rect_height;
	int rotate_enable = 0;
	tr_mode rt_mode;
	if(framebuffersink->rotate_angle_property > 0)
	{
		rotate_enable = 1;
		rt_mode = framebuffersink->rotate_angle_property;
	}
	else
	{
		rotate_enable = 0;
	}

	memset(&trans_info, 0, sizeof(tr_info));
	gst_memory_map(mem, &mapinfo, GST_MAP_READ);

    memset(&luapiconfig, 0, sizeof(luapiconfig));

	phymem_start = SunxiMemGetPhysicAddressCpu(ops, mapinfo.data);

	SunxiMemGetActualSize(ops,&rect_width,&rect_height);

#ifdef __SUNXI_DISPLAY2__
	if (format == GST_VIDEO_FORMAT_Y444) {
	  luapiconfig.layerConfig.info.fb.addr[0] = (unsigned long long )phymem_start;
	  luapiconfig.layerConfig.info.fb.addr[1] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[1]);
	  luapiconfig.layerConfig.info.fb.addr[2] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[2]);
	  luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[2].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
	  luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height;
	  luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height;
	  luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV444_P;
	  rotate_enable = 0;
	}
	else if (format == GST_VIDEO_FORMAT_NV12
		|| format == GST_VIDEO_FORMAT_NV21) {
	  if(format == GST_VIDEO_FORMAT_NV12)
		luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_SP_UVUV;
	  else
		luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
	  luapiconfig.layerConfig.info.fb.addr[0] = (unsigned long long )phymem_start;
	  luapiconfig.layerConfig.info.fb.addr[1] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[1]);

	  luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[1].width = (framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8))/2;

	  luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
	  luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
	}
	else {
	  luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_P;
	  luapiconfig.layerConfig.info.fb.addr[0] = (unsigned long long )phymem_start;
	  if (format == GST_VIDEO_FORMAT_I420) {
		luapiconfig.layerConfig.info.fb.addr[1] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[1]);
		luapiconfig.layerConfig.info.fb.addr[2] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[2]);
		luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
		luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
		luapiconfig.layerConfig.info.fb.size[2].width= framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
	  }
	  else {
		/* GST_VIDEO_FORMAT_YV12 */
		luapiconfig.layerConfig.info.fb.addr[1] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[2]);
		luapiconfig.layerConfig.info.fb.addr[2] = (unsigned long long )(phymem_start + framebuffersink->overlay_plane_offset[1]);

		luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
		luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
		luapiconfig.layerConfig.info.fb.size[2].width= framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
	  }
	}

	if(rotate_enable)
	{
		width_align = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width);
		height_align = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].height);
		if(sunxifbsink->rotate_addr_phy[0] == NULL && sunxifbsink->rotate_addr_phy[1] == NULL)
		{
			unsigned int buffer_len = width_align*height_align*3/2;
			sunxifbsink->rotate_addr_phy[0] = (char *)SunxiMemPalloc(ops,buffer_len);
			if(sunxifbsink->rotate_addr_phy[0] == NULL)
			{
				GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when rotate!\n");
                gst_memory_unmap(mem, &mapinfo);
				return GST_FLOW_ERROR;
			}
			sunxifbsink->rotate_addr_phy[1] = (char *)SunxiMemPalloc(ops,buffer_len);
			if(sunxifbsink->rotate_addr_phy[1] == NULL)
			{
				SunxiMemPfree(ops,sunxifbsink->rotate_addr_phy[0]);
				GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when rotate!\n");
                gst_memory_unmap(mem, &mapinfo);
				return GST_FLOW_ERROR;

			}
			memset(sunxifbsink->rotate_addr_phy[0],0,buffer_len);
			memset(sunxifbsink->rotate_addr_phy[1],0,buffer_len);
			SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[0],buffer_len);
			SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[1],buffer_len);
		}

		trans_info.mode = rt_mode;
		trans_info.src_frame.fmt = TR_FORMAT_YUV420_P;
		trans_info.src_frame.laddr[0] = (unsigned long)phymem_start;
		trans_info.src_frame.laddr[1] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[1];
		trans_info.src_frame.laddr[2] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[2];

		trans_info.src_frame.pitch[0] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width);
		trans_info.src_frame.pitch[1] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width)/2;
		trans_info.src_frame.pitch[2] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width)/2;
		trans_info.src_frame.height[0] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].height);
		trans_info.src_frame.height[1] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[1].height)/2;
		trans_info.src_frame.height[2] = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[2].height)/2;

		trans_info.src_rect.x = 0;
		trans_info.src_rect.y = 0;
		trans_info.src_rect.w = rect_width;
		trans_info.src_rect.h = rect_height;

		trans_info.dst_frame.fmt = TR_FORMAT_YUV420_P;
		trans_info.dst_frame.laddr[0] = (unsigned long)SunxiMemGetPhysicAddressCpu(ops,(void *)sunxifbsink->rotate_addr_phy[(++m)%2]);
		trans_info.dst_frame.laddr[1] = trans_info.dst_frame.laddr[0] + width_align * height_align;
		trans_info.dst_frame.laddr[2] = trans_info.dst_frame.laddr[0] + width_align * height_align*5/4;

		if(trans_info.mode == TR_ROT_180)
		{
			trans_info.dst_frame.pitch[0] = width_align;
			trans_info.dst_frame.pitch[1] = width_align/2;
			trans_info.dst_frame.pitch[2] = width_align/2;

			trans_info.dst_frame.height[0] = height_align;
			trans_info.dst_frame.height[1] = height_align/2;
			trans_info.dst_frame.height[2] = height_align/2;

			trans_info.dst_rect.x = 0;
			trans_info.dst_rect.y = 0;
			trans_info.dst_rect.w = width_align;
			trans_info.dst_rect.h = height_align;
		}
		else
		{
			trans_info.dst_frame.pitch[0] = height_align;
			trans_info.dst_frame.pitch[1] = height_align/2;
			trans_info.dst_frame.pitch[2] = height_align/2;

			trans_info.dst_frame.height[0] = width_align;
			trans_info.dst_frame.height[1] = width_align/2;
			trans_info.dst_frame.height[2] = width_align/2;

			trans_info.dst_rect.x = 0;
			trans_info.dst_rect.y = 0;
			trans_info.dst_rect.w = height_align;
			trans_info.dst_rect.h = width_align;
		}

        if(sunxifbsink->fd_transform > 0)
            hwRotateVideoPicture(sunxifbsink, &trans_info);

		luapiconfig.layerConfig.info.fb.addr[0] = (unsigned long long )SunxiMemGetPhysicAddressCpu(ops,sunxifbsink->rotate_addr_phy[0]);
		luapiconfig.layerConfig.info.fb.addr[1] = (unsigned long long )trans_info.dst_frame.laddr[1];
		luapiconfig.layerConfig.info.fb.addr[2] = (unsigned long long )trans_info.dst_frame.laddr[2];

		luapiconfig.layerConfig.info.fb.size[0].width = trans_info.dst_frame.pitch[0];
		luapiconfig.layerConfig.info.fb.size[1].width = trans_info.dst_frame.pitch[1];
		luapiconfig.layerConfig.info.fb.size[2].width = trans_info.dst_frame.pitch[2];

		luapiconfig.layerConfig.info.fb.size[0].height = trans_info.dst_frame.height[0];
		luapiconfig.layerConfig.info.fb.size[1].height = trans_info.dst_frame.height[1];
		luapiconfig.layerConfig.info.fb.size[2].height = trans_info.dst_frame.height[2];
	}
	//initialize layer info
	luapiconfig.layerConfig.info.mode = LAYER_MODE_BUFFER;
	luapiconfig.layerConfig.info.zorder = 11;
	luapiconfig.layerConfig.info.alpha_mode = 1;
	luapiconfig.layerConfig.info.alpha_value = 0xff;

	luapiconfig.layerConfig.info.fb.crop.x = 0;
	luapiconfig.layerConfig.info.fb.crop.y = 0;
	if(rotate_enable && (trans_info.mode == TR_ROT_90 || trans_info.mode == TR_ROT_270))
	{
		luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)rect_height << 32;
		luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)rect_width << 32;
	}
	else
	{
		luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)rect_width << 32;
		luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)rect_height << 32;
	}
	luapiconfig.layerConfig.info.fb.color_space = (framebuffersink->video_rectangle.h < 720) ? DISP_BT601 : DISP_BT709;

	luapiconfig.layerConfig.info.screen_win.x = framebuffersink->video_rectangle.x;
	luapiconfig.layerConfig.info.screen_win.y = framebuffersink->video_rectangle.y;
	luapiconfig.layerConfig.info.screen_win.width = framebuffersink->video_rectangle.w;
	luapiconfig.layerConfig.info.screen_win.height = framebuffersink->video_rectangle.h;

	luapiconfig.layerConfig.enable = TRUE;
	luapiconfig.layerConfig.layer_id = sunxifbsink->layer_id;
	luapiconfig.layerConfig.channel = sunxifbsink->framebuffer_id;

	luapiconfig.layerConfig.info.fb.flags= DISP_BF_NORMAL;
	luapiconfig.layerConfig.info.fb.scan= DISP_SCAN_PROGRESSIVE;
#else
    DispGetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
                                        sunxifbsink->framebuffer_id, 1, &luapiconfig);

    if (format == GST_VIDEO_FORMAT_Y444) {
      luapiconfig.layerConfig.fb.addr[0] = (unsigned int)phymem_start;
      luapiconfig.layerConfig.fb.addr[1] = (unsigned int)phymem_start
          + framebuffersink->overlay_plane_offset[1];
      luapiconfig.layerConfig.fb.addr[2] = (unsigned int)phymem_start
          + framebuffersink->overlay_plane_offset[2];
      luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
      luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV444_P;
    }
    else if (format == GST_VIDEO_FORMAT_NV12
        || format == GST_VIDEO_FORMAT_NV21) {
      luapiconfig.layerConfig.fb.addr[0] = (unsigned int)phymem_start;
      luapiconfig.layerConfig.fb.addr[1] = (unsigned int)phymem_start
          + framebuffersink->overlay_plane_offset[1];

      luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
      luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height/2;
      if(format == GST_VIDEO_FORMAT_NV12)
        luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_SP_UVUV;
      else
        luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
    }
    else {
      luapiconfig.layerConfig.fb.addr[0] = (unsigned int)phymem_start;
      if (format == GST_VIDEO_FORMAT_I420) {
        luapiconfig.layerConfig.fb.addr[1] = (unsigned int)phymem_start + framebuffersink->overlay_plane_offset[1];
        luapiconfig.layerConfig.fb.addr[2] = (unsigned int)phymem_start + framebuffersink->overlay_plane_offset[2];
        luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
        luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      }
      else {
        /* GST_VIDEO_FORMAT_YV12 */
        luapiconfig.layerConfig.fb.addr[1] = (unsigned int)phymem_start + framebuffersink->overlay_plane_offset[2];
        luapiconfig.layerConfig.fb.addr[2] = (unsigned int)phymem_start + framebuffersink->overlay_plane_offset[1];

        luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
        luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      }
        luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_P;
    }

	if(rotate_enable)
	{
		width_align = ALIGN_32B(luapiconfig.layerConfig.fb.size.width);
		height_align = ALIGN_32B(luapiconfig.layerConfig.fb.size.height);
		if(sunxifbsink->rotate_addr_phy[0] == NULL && sunxifbsink->rotate_addr_phy[1] == NULL)
		{
			unsigned int buffer_len = width_align*height_align*3/2;
			sunxifbsink->rotate_addr_phy[0] = (char *)SunxiMemPalloc(ops,buffer_len);
			if(sunxifbsink->rotate_addr_phy[0] == NULL)
			{
				GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when rotate!\n");
                gst_memory_unmap(mem, &mapinfo);
				return GST_FLOW_ERROR;
			}
			sunxifbsink->rotate_addr_phy[1] = (char *)SunxiMemPalloc(ops,buffer_len);
			if(sunxifbsink->rotate_addr_phy[1] == NULL)
			{
				SunxiMemPfree(ops,sunxifbsink->rotate_addr_phy[0]);
				GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when rotate!\n");
                gst_memory_unmap(mem, &mapinfo);
				return GST_FLOW_ERROR;
			}
			memset(sunxifbsink->rotate_addr_phy[0], 0, buffer_len);
			memset(sunxifbsink->rotate_addr_phy[1], 0, buffer_len);
			SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[0], buffer_len);
			SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[1], buffer_len);
		}

		trans_info.mode = rt_mode;
		trans_info.src_frame.fmt = TR_FORMAT_YUV420_P;
		trans_info.src_frame.laddr[0] = (unsigned long)(unsigned int)phymem_start;
		trans_info.src_frame.laddr[1] = (unsigned long)luapiconfig.layerConfig.fb.addr[1];
		trans_info.src_frame.laddr[2] = (unsigned long)luapiconfig.layerConfig.fb.addr[2];

		trans_info.src_frame.pitch[0] = ALIGN_32B(luapiconfig.layerConfig.fb.size.width);
		trans_info.src_frame.pitch[1] = ALIGN_32B(luapiconfig.layerConfig.fb.size.width)/2;
		trans_info.src_frame.pitch[2] = ALIGN_32B(luapiconfig.layerConfig.fb.size.width)/2;
		trans_info.src_frame.height[0] = ALIGN_32B(luapiconfig.layerConfig.fb.size.height);
		trans_info.src_frame.height[1] = ALIGN_32B(luapiconfig.layerConfig.fb.size.height)/2;
		trans_info.src_frame.height[2] = ALIGN_32B(luapiconfig.layerConfig.fb.size.height)/2;

		trans_info.src_rect.x = 0;
		trans_info.src_rect.y = 0;
		trans_info.src_rect.w = rect_width;
		trans_info.src_rect.h = rect_height;

		trans_info.dst_frame.fmt = TR_FORMAT_YUV420_P;
		trans_info.dst_frame.laddr[0] = (unsigned long)SunxiMemGetPhysicAddressCpu(ops,(void *)sunxifbsink->rotate_addr_phy[(++m)%2]);
		trans_info.dst_frame.laddr[1] = trans_info.dst_frame.laddr[0] + width_align * height_align;
		trans_info.dst_frame.laddr[2] = trans_info.dst_frame.laddr[0] + width_align * height_align*5/4;

		if(trans_info.mode == TR_ROT_180)
		{
			trans_info.dst_frame.pitch[0] = width_align;
			trans_info.dst_frame.pitch[1] = width_align/2;
			trans_info.dst_frame.pitch[2] = width_align/2;

			trans_info.dst_frame.height[0] = height_align;
			trans_info.dst_frame.height[1] = height_align/2;
			trans_info.dst_frame.height[2] = height_align/2;

			trans_info.dst_rect.x = 0;
			trans_info.dst_rect.y = 0;
			trans_info.dst_rect.w = width_align;
			trans_info.dst_rect.h = height_align;
		}
		else
		{
			trans_info.dst_frame.pitch[0] = height_align;
			trans_info.dst_frame.pitch[1] = height_align/2;
			trans_info.dst_frame.pitch[2] = height_align/2;

			trans_info.dst_frame.height[0] = width_align;
			trans_info.dst_frame.height[1] = width_align/2;
			trans_info.dst_frame.height[2] = width_align/2;

			trans_info.dst_rect.x = 0;
			trans_info.dst_rect.y = 0;
			trans_info.dst_rect.w = height_align;
			trans_info.dst_rect.h = width_align;
		}

        if(sunxifbsink->fd_transform > 0)
            hwRotateVideoPicture(sunxifbsink, &trans_info);

		luapiconfig.layerConfig.fb.addr[0] = (unsigned int)SunxiMemGetPhysicAddressCpu(ops,sunxifbsink->rotate_addr_phy[0]);
		luapiconfig.layerConfig.fb.addr[1] = (unsigned int)trans_info.dst_frame.laddr[1];
		luapiconfig.layerConfig.fb.addr[2] = (unsigned int)trans_info.dst_frame.laddr[2];

		luapiconfig.layerConfig.fb.size.width = trans_info.dst_frame.pitch[0];

		luapiconfig.layerConfig.fb.size.height = trans_info.dst_frame.height[0];
	}
/*
	if(rotate_enable && (trans_info.mode == TR_ROT_90 || trans_info.mode == TR_ROT_270))
	{
		luapiconfig.layerConfig.fb.crop.width = (unsigned long long)rect_height << 32;
		luapiconfig.layerConfig.fb.crop.height = (unsigned long long)rect_width << 32;
	}
	else
	{
		luapiconfig.layerConfig.fb.crop.width = (unsigned long long)rect_width << 32;
		luapiconfig.layerConfig.fb.crop.height = (unsigned long long)rect_height << 32;
	}
*/
    /* Source size (can be cropped) */
    luapiconfig.layerConfig.fb.src_win.x = 0;
    luapiconfig.layerConfig.fb.src_win.y = 0;
    luapiconfig.layerConfig.fb.src_win.width = framebuffersink->videosink.width;
    luapiconfig.layerConfig.fb.src_win.height = framebuffersink->videosink.height;

    /* Display position and size */
    luapiconfig.layerConfig.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.screen_win.height = framebuffersink->video_rectangle.h;

    luapiconfig.layerConfig.alpha_mode = 0;
    luapiconfig.layerConfig.fb.pre_multiply = 0;
    luapiconfig.layerConfig.alpha_value = 0xff;
    luapiconfig.layerConfig.zorder = 3;
    luapiconfig.layerConfig.mode = DISP_LAYER_WORK_MODE_SCALER;
    luapiconfig.layerConfig.pipe = 0;

#endif

    if (DispSetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                1, &luapiconfig) < 0){
        gst_memory_unmap(mem, &mapinfo);
		return FALSE;
    }

	gst_sunxifbsink_show_layer(sunxifbsink);
    gst_memory_unmap(mem, &mapinfo);
	return GST_FLOW_OK;
}

static bool reset_video_rectangle_flag = 1;

static GstFlowReturn
gst_sunxifbsink_show_overlay_yuv_planar (GstFramebufferSink *framebuffersink,
    guintptr framebuffer_offset, GstVideoFormat format)
{
    GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
    luapi_layer_config luapiconfig;
    struct SunxiMemOpsS* ops =  GetMemAdapterOpsS();
    int nRotateDegree = framebuffersink->rotate_angle_property;
    int rotate_enable = 0;
    unsigned int width_align;
    unsigned int height_align;
    g2d_blt_h    blit;
    static int   m = 0;

    if(nRotateDegree > 0)
    {
        rotate_enable = 1;
    }

    memset(&luapiconfig, 0, sizeof(luapiconfig));

#ifdef __SUNXI_DISPLAY2__
    if (format == GST_VIDEO_FORMAT_Y444) {
      luapiconfig.layerConfig.info.fb.addr[0] = framebuffer_offset;
      luapiconfig.layerConfig.info.fb.addr[1] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[1];
      luapiconfig.layerConfig.info.fb.addr[2] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[2];
	  luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
      luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->overlay_scanline_stride[0]
		/ (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
		0, 8)
		* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[2].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
      luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height;
	  luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height;
      luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV444_P;
    }
    else if (format == GST_VIDEO_FORMAT_NV12
        || format == GST_VIDEO_FORMAT_NV21) {
      if(format == GST_VIDEO_FORMAT_NV12)
	luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_SP_UVUV;
	  else
		luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
      luapiconfig.layerConfig.info.fb.addr[0] = framebuffer_offset;
      luapiconfig.layerConfig.info.fb.addr[1] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[1];

	  luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.info.fb.size[1].width = (framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8))/2;

      luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
	  luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
    }
    else {
	  luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV420_P;
      luapiconfig.layerConfig.info.fb.addr[0] = framebuffer_offset;
      if (format == GST_VIDEO_FORMAT_I420) {
        luapiconfig.layerConfig.info.fb.addr[1] = framebuffer_offset + framebuffersink->overlay_plane_offset[1];
        luapiconfig.layerConfig.info.fb.addr[2] = framebuffer_offset + framebuffersink->overlay_plane_offset[2];
		luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
		luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
		luapiconfig.layerConfig.info.fb.size[2].width= framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
      }
      else {
        /* GST_VIDEO_FORMAT_YV12 */
        luapiconfig.layerConfig.info.fb.addr[1] = framebuffer_offset + framebuffersink->overlay_plane_offset[2];
        luapiconfig.layerConfig.info.fb.addr[2] = framebuffer_offset + framebuffersink->overlay_plane_offset[1];

		luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
		luapiconfig.layerConfig.info.fb.size[1].width = framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[1].height = framebuffersink->videosink.height/2;
		luapiconfig.layerConfig.info.fb.size[2].width= framebuffersink->videosink.width/2;
		luapiconfig.layerConfig.info.fb.size[2].height = framebuffersink->videosink.height/2;
      }
    }

#ifdef __SUNXI_G2D_ROTATE__
    if(rotate_enable)
    {
        width_align = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width);
        height_align = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].height);
        if(sunxifbsink->rotate_addr_phy[0] == NULL && sunxifbsink->rotate_addr_phy[1] == NULL)
        {
            unsigned int buffer_len = width_align*height_align*3/2;
            sunxifbsink->rotate_addr_phy[0] = (char *)SunxiMemPalloc(ops,buffer_len);
            if(sunxifbsink->rotate_addr_phy[0] == NULL)
            {
                GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when g2d rotate!\n");
                return GST_FLOW_ERROR;
            }
            sunxifbsink->rotate_addr_phy[1] = (char *)SunxiMemPalloc(ops,buffer_len);
            if(sunxifbsink->rotate_addr_phy[1] == NULL)
            {
                SunxiMemPfree(ops,sunxifbsink->rotate_addr_phy[0]);
                GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "-->no physical memory when rotate!\n");
                return GST_FLOW_ERROR;
            }
            memset(sunxifbsink->rotate_addr_phy[0],0,buffer_len);
            memset(sunxifbsink->rotate_addr_phy[1],0,buffer_len);
            SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[0],buffer_len);
            SunxiMemFlushCache(ops,sunxifbsink->rotate_addr_phy[1],buffer_len);
        }

        memset(&blit, 0, sizeof(g2d_blt_h));

        if(format == GST_VIDEO_FORMAT_YV12)
        {
            blit.src_image_h.format = G2D_FORMAT_YUV420_PLANAR;//g2d_fmt_enh
            blit.dst_image_h.format = G2D_FORMAT_YUV420_PLANAR;
            blit.src_image_h.laddr[0] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[0];
            blit.src_image_h.laddr[1] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[1];
            blit.src_image_h.laddr[2] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[2];
            blit.dst_image_h.laddr[0] = (unsigned long)SunxiMemGetPhysicAddressCpu(ops,(void *)sunxifbsink->rotate_addr_phy[(++m)%2]);
            blit.dst_image_h.laddr[1] = blit.dst_image_h.laddr[0] + width_align * height_align;
            blit.dst_image_h.laddr[2] = blit.dst_image_h.laddr[0] + width_align * height_align * 5/4;
        }
        else if(format == GST_VIDEO_FORMAT_NV21)
        {
            blit.src_image_h.format = G2D_FORMAT_YUV420UVC_U1V1U0V0;//g2d_fmt_enh
            blit.dst_image_h.format = G2D_FORMAT_YUV420UVC_U1V1U0V0;
            blit.src_image_h.laddr[0] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[0];
            blit.src_image_h.laddr[1] = (unsigned long)luapiconfig.layerConfig.info.fb.addr[1];
            blit.src_image_h.laddr[2] = 0;
            blit.dst_image_h.laddr[0] = (unsigned long)SunxiMemGetPhysicAddressCpu(ops,(void *)sunxifbsink->rotate_addr_phy[(++m)%2]);
            blit.dst_image_h.laddr[1] = blit.dst_image_h.laddr[0] + width_align * height_align;
        }
        else
        {
            GST_ERROR_OBJECT(sunxifbsink, "the format[0x%x] is not support by g2d driver", format);
            return GST_FLOW_ERROR;
        }

        switch(nRotateDegree)
        {
            case 0:
                blit.flag_h = G2D_ROT_0;
                break;
            case 1:
                blit.flag_h = G2D_ROT_90;
                break;
            case 2:
                blit.flag_h = G2D_ROT_180;
                break;
            case 3:
                blit.flag_h = G2D_ROT_270;
                break;
            case 4:
                blit.flag_h = G2D_ROT_H;
                break;
            case 6:
                blit.flag_h = G2D_ROT_V;
                break;
            default:
                GST_ERROR_OBJECT(sunxifbsink, "fatal error! rot_angle[%d] is invalid!", nRotateDegree);
                blit.flag_h = G2D_BLT_NONE_H;
                break;
        }

        blit.src_image_h.bbuff = 1;
        blit.src_image_h.use_phy_addr = 1;
        blit.src_image_h.color = 0xff;
        blit.src_image_h.width = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].width);
        blit.src_image_h.height = ALIGN_32B(luapiconfig.layerConfig.info.fb.size[0].height);
        blit.src_image_h.align[0] = 0;
        blit.src_image_h.align[1] = 0;
        blit.src_image_h.align[2] = 0;
        blit.src_image_h.clip_rect.x = 0;
        blit.src_image_h.clip_rect.y = 0;
        blit.src_image_h.clip_rect.w = luapiconfig.layerConfig.info.fb.size[0].width;
        blit.src_image_h.clip_rect.h = luapiconfig.layerConfig.info.fb.size[0].height;
        blit.src_image_h.gamut = G2D_BT709;
        blit.src_image_h.bpremul = 0;
        blit.src_image_h.alpha = 0xff;
        blit.src_image_h.mode = G2D_GLOBAL_ALPHA;
        blit.dst_image_h.bbuff = 1;
        blit.dst_image_h.use_phy_addr = 1;
        blit.dst_image_h.color = 0xff;
        blit.dst_image_h.align[0] = 0;
        blit.dst_image_h.align[1] = 0;
        blit.dst_image_h.align[2] = 0;
        blit.dst_image_h.gamut = G2D_BT709;
        blit.dst_image_h.bpremul = 0;
        blit.dst_image_h.alpha = 0xff;
        blit.dst_image_h.mode = G2D_GLOBAL_ALPHA;

        if(blit.flag_h == G2D_ROT_90 || blit.flag_h == G2D_ROT_270)
        {
            blit.dst_image_h.width = height_align;
            blit.dst_image_h.height = width_align;
            blit.dst_image_h.clip_rect.x = 0;
            blit.dst_image_h.clip_rect.y = 0;
            blit.dst_image_h.clip_rect.w = luapiconfig.layerConfig.info.fb.size[0].height;
            blit.dst_image_h.clip_rect.h = luapiconfig.layerConfig.info.fb.size[0].width;
        }
        else
        {
            blit.dst_image_h.width = width_align;
            blit.dst_image_h.height = height_align;
            blit.dst_image_h.clip_rect.x = 0;
            blit.dst_image_h.clip_rect.y = 0;
            blit.dst_image_h.clip_rect.w = luapiconfig.layerConfig.info.fb.size[0].width;
            blit.dst_image_h.clip_rect.h = luapiconfig.layerConfig.info.fb.size[0].height;
        }
        if (ioctl(sunxifbsink->fd_g2d,G2D_CMD_BITBLT_H,(unsigned long)&blit) < 0)
        {
            GST_SUNXIFBSINK_ERROR_OBJECT(sunxifbsink, "---->g2d G2D_CMD_BITBLT_H fail!");
            return GST_FLOW_ERROR;
        }

        luapiconfig.layerConfig.info.fb.addr[0] = blit.dst_image_h.laddr[0];
        luapiconfig.layerConfig.info.fb.addr[1] = blit.dst_image_h.laddr[1];
        luapiconfig.layerConfig.info.fb.addr[2] = blit.dst_image_h.laddr[2];

        luapiconfig.layerConfig.info.fb.size[0].width = blit.dst_image_h.width;
        luapiconfig.layerConfig.info.fb.size[0].height = blit.dst_image_h.height;
        luapiconfig.layerConfig.info.fb.size[1].width = blit.dst_image_h.width/2;
        luapiconfig.layerConfig.info.fb.size[1].height = blit.dst_image_h.height/2;
        luapiconfig.layerConfig.info.fb.size[2].width= blit.dst_image_h.width/2;
        luapiconfig.layerConfig.info.fb.size[2].height = blit.dst_image_h.height/2;
    }
#endif

    //initialize layer info
    luapiconfig.layerConfig.info.mode = LAYER_MODE_BUFFER;
    luapiconfig.layerConfig.info.zorder = 11;
    luapiconfig.layerConfig.info.alpha_mode = 1;
    luapiconfig.layerConfig.info.alpha_value = 0xff;

    luapiconfig.layerConfig.info.fb.crop.x = 0;
    luapiconfig.layerConfig.info.fb.crop.y = 0;
#ifdef __SUNXI_G2D_ROTATE__
    if(rotate_enable && (blit.flag_h == G2D_ROT_90 || blit.flag_h == G2D_ROT_270))
    {
        luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)framebuffersink->videosink.height << 32;
        luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)framebuffersink->videosink.width << 32;
        if(reset_video_rectangle_flag)
        {
            GstVideoRectangle screen_video_rectangle, dst_video_rectangle;
            screen_video_rectangle.x = 0;
            screen_video_rectangle.y = 0;
            screen_video_rectangle.w = GST_VIDEO_INFO_WIDTH(&framebuffersink->screen_info);
            screen_video_rectangle.h = GST_VIDEO_INFO_HEIGHT(&framebuffersink->screen_info);
            dst_video_rectangle.x = 0;
            dst_video_rectangle.y = 0;
            dst_video_rectangle.w = blit.dst_image_h.width;
            dst_video_rectangle.h = blit.dst_image_h.height;
            /* Center it. */
            gst_video_sink_center_rect (dst_video_rectangle, screen_video_rectangle, &framebuffersink->video_rectangle, FALSE);
            reset_video_rectangle_flag = 0;
        }
    }
    else
#endif
    {
        luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)framebuffersink->videosink.width << 32;
        luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)framebuffersink->videosink.height << 32;
    }
    luapiconfig.layerConfig.info.fb.color_space = (framebuffersink->video_rectangle.h < 720) ? DISP_BT601 : DISP_BT709;

    luapiconfig.layerConfig.info.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.info.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.info.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.info.screen_win.height = framebuffersink->video_rectangle.h;

	luapiconfig.layerConfig.enable = TRUE;
	luapiconfig.layerConfig.layer_id = sunxifbsink->layer_id;
	luapiconfig.layerConfig.channel = sunxifbsink->framebuffer_id;

	luapiconfig.layerConfig.info.fb.flags= DISP_BF_NORMAL;
	luapiconfig.layerConfig.info.fb.scan= DISP_SCAN_PROGRESSIVE;
#else
    DispGetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                sunxifbsink->framebuffer_id, 1, &luapiconfig);

    if (format == GST_VIDEO_FORMAT_Y444) {
      luapiconfig.layerConfig.fb.addr[0] = framebuffer_offset;
      luapiconfig.layerConfig.fb.addr[1] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[1];
      luapiconfig.layerConfig.fb.addr[2] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[2];
	  luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
	  luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV444_P;
    }
    else if (format == GST_VIDEO_FORMAT_NV12
        || format == GST_VIDEO_FORMAT_NV21) {
      luapiconfig.layerConfig.fb.addr[0] = framebuffer_offset;
      luapiconfig.layerConfig.fb.addr[1] = framebuffer_offset
          + framebuffersink->overlay_plane_offset[1];

	  luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
      luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height/2;
      if(format == GST_VIDEO_FORMAT_NV12)
	luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_SP_UVUV;
	  else
		luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_SP_VUVU;
    }
    else {
      luapiconfig.layerConfig.fb.addr[0] = framebuffer_offset;
      if (format == GST_VIDEO_FORMAT_I420) {
        luapiconfig.layerConfig.fb.addr[1] = framebuffer_offset + framebuffersink->overlay_plane_offset[1];
        luapiconfig.layerConfig.fb.addr[2] = framebuffer_offset + framebuffersink->overlay_plane_offset[2];
		luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      }
      else {
        /* GST_VIDEO_FORMAT_YV12 */
        luapiconfig.layerConfig.fb.addr[1] = framebuffer_offset + framebuffersink->overlay_plane_offset[2];
        luapiconfig.layerConfig.fb.addr[2] = framebuffer_offset + framebuffersink->overlay_plane_offset[1];

		luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo,
        0, 8)* GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
		luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
      }
        luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV420_P;
    }

    /* Source size (can be cropped) */
    luapiconfig.layerConfig.fb.src_win.x = 0;
    luapiconfig.layerConfig.fb.src_win.y = 0;
    luapiconfig.layerConfig.fb.src_win.width = framebuffersink->videosink.width;
    luapiconfig.layerConfig.fb.src_win.height = framebuffersink->videosink.height;

    /* Display position and size */
    luapiconfig.layerConfig.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.screen_win.height = framebuffersink->video_rectangle.h;

    luapiconfig.layerConfig.alpha_mode = 0;
    luapiconfig.layerConfig.fb.pre_multiply = 0;
    luapiconfig.layerConfig.alpha_value = 0xff;
    luapiconfig.layerConfig.zorder = 3;
    luapiconfig.layerConfig.mode = DISP_LAYER_WORK_MODE_SCALER;
    luapiconfig.layerConfig.pipe = 0;
#endif

    if (DispSetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                1, &luapiconfig) < 0)
		return FALSE;

    gst_sunxifbsink_show_layer(sunxifbsink);

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_sunxifbsink_show_overlay_yuv_packed (GstFramebufferSink *framebuffersink,
    guintptr framebuffer_offset, GstVideoFormat format)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
    luapi_layer_config luapiconfig;

    memset(&luapiconfig, 0, sizeof(luapiconfig));

#ifdef __SUNXI_DISPLAY2__
    luapiconfig.layerConfig.info.fb.addr[0] = framebuffer_offset;
    luapiconfig.layerConfig.info.fb.size[0].height = framebuffersink->videosink.height;
	luapiconfig.layerConfig.info.fb.size[0].width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo, 0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);

    if (format == GST_VIDEO_FORMAT_AYUV) {
      luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV444_P;
      GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "---->AYUV");
    }
    else {
      luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_YUV422_P;
      if (format == GST_VIDEO_FORMAT_YUY2)
       GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "---->YUY2");
    }

    //initialize layer info
	luapiconfig.layerConfig.info.mode = LAYER_MODE_BUFFER;
	luapiconfig.layerConfig.info.zorder = 11;
	luapiconfig.layerConfig.info.alpha_mode = 1;
	luapiconfig.layerConfig.info.alpha_value = 0xff;

	luapiconfig.layerConfig.info.fb.crop.x = 0;
    luapiconfig.layerConfig.info.fb.crop.y = 0;
    luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)framebuffersink->videosink.width << 32;
    luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)framebuffersink->videosink.height << 32;
	luapiconfig.layerConfig.info.fb.color_space = (framebuffersink->video_rectangle.h < 720) ? DISP_BT601 : DISP_BT709;

    luapiconfig.layerConfig.info.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.info.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.info.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.info.screen_win.height = framebuffersink->video_rectangle.h;

	luapiconfig.layerConfig.enable = TRUE;
	luapiconfig.layerConfig.layer_id = sunxifbsink->layer_id;
	luapiconfig.layerConfig.channel = sunxifbsink->framebuffer_id;

	luapiconfig.layerConfig.info.fb.flags= DISP_BF_NORMAL;
	luapiconfig.layerConfig.info.fb.scan= DISP_SCAN_PROGRESSIVE;
#else
    DispGetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                sunxifbsink->framebuffer_id, 1, &luapiconfig);

    luapiconfig.layerConfig.fb.addr[0] = framebuffer_offset;
    luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0]
        / (GST_VIDEO_FORMAT_INFO_SCALE_WIDTH (framebuffersink->video_info.finfo, 0, 8)
        * GST_VIDEO_INFO_COMP_PSTRIDE (&framebuffersink->video_info, 0) / 8);
    luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;

    if (format == GST_VIDEO_FORMAT_AYUV) {
      luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV444_P;
      GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "---->AYUV");
    }
    else {
      luapiconfig.layerConfig.fb.format = DISP_FORMAT_YUV422_P;
      if (format == GST_VIDEO_FORMAT_YUY2)
       GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "---->YUY2");
    }

    /* Source size (can be cropped) */
    luapiconfig.layerConfig.fb.src_win.x = 0;
    luapiconfig.layerConfig.fb.src_win.y = 0;
    luapiconfig.layerConfig.fb.src_win.width = framebuffersink->videosink.width;
    luapiconfig.layerConfig.fb.src_win.height = framebuffersink->videosink.height;

    /* Display position and size */
    luapiconfig.layerConfig.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.screen_win.height = framebuffersink->video_rectangle.h;

    luapiconfig.layerConfig.alpha_mode = 0;
    luapiconfig.layerConfig.fb.pre_multiply = 0;
    luapiconfig.layerConfig.alpha_value = 0xff;
    luapiconfig.layerConfig.zorder = 3;
    luapiconfig.layerConfig.mode = DISP_LAYER_WORK_MODE_SCALER;
    luapiconfig.layerConfig.pipe = 0;
#endif

    if (DispSetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                1, &luapiconfig) < 0)
		return FALSE;

    gst_sunxifbsink_show_layer(sunxifbsink);

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_sunxifbsink_show_overlay_bgrx32 (GstFramebufferSink *framebuffersink,
    guintptr framebuffer_offset)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
    luapi_layer_config luapiconfig;

	GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->sunxisink_show_overlay_bgrx32");

    memset(&luapiconfig, 0, sizeof(luapiconfig));

#ifdef __SUNXI_DISPLAY2__
    /* BGRX layer. */
	luapiconfig.layerConfig.info.mode = LAYER_MODE_BUFFER;
	luapiconfig.layerConfig.info.zorder = 11;
	luapiconfig.layerConfig.info.alpha_mode = 1;
	luapiconfig.layerConfig.info.alpha_value = 0xff;

    luapiconfig.layerConfig.info.fb.addr[0] = framebuffer_offset;
    luapiconfig.layerConfig.info.fb.size[sunxifbsink->framebuffer_id].width = framebuffersink->overlay_scanline_stride[0] >> 2;
    luapiconfig.layerConfig.info.fb.size[sunxifbsink->framebuffer_id].height = framebuffersink->videosink.height;
    luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_ARGB_8888;

	luapiconfig.layerConfig.info.fb.crop.x = 0;
    luapiconfig.layerConfig.info.fb.crop.y = 0;
    luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)framebuffersink->videosink.width << 32;
    luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)framebuffersink->videosink.height << 32;
	luapiconfig.layerConfig.info.fb.color_space = (framebuffersink->video_rectangle.h < 720) ? DISP_BT601 : DISP_BT709;

    luapiconfig.layerConfig.info.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.info.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.info.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.info.screen_win.height = framebuffersink->video_rectangle.h;

	luapiconfig.layerConfig.enable = TRUE;
	luapiconfig.layerConfig.layer_id = sunxifbsink->layer_id;
	luapiconfig.layerConfig.channel = sunxifbsink->framebuffer_id;
	luapiconfig.layerConfig.info.fb.flags= DISP_BF_NORMAL;
	luapiconfig.layerConfig.info.fb.scan= DISP_SCAN_PROGRESSIVE;
#else

    DispGetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                sunxifbsink->framebuffer_id, 1, &luapiconfig);

    luapiconfig.layerConfig.fb.addr[0] = framebuffer_offset;
    luapiconfig.layerConfig.fb.size.width = framebuffersink->overlay_scanline_stride[0] >> 2;
    luapiconfig.layerConfig.fb.size.height = framebuffersink->videosink.height;
    luapiconfig.layerConfig.fb.format = DISP_FORMAT_ARGB_8888;

    /* Source size (can be cropped) */
    luapiconfig.layerConfig.fb.src_win.x = 0;
    luapiconfig.layerConfig.fb.src_win.y = 0;
    luapiconfig.layerConfig.fb.src_win.width = framebuffersink->videosink.width;
    luapiconfig.layerConfig.fb.src_win.height = framebuffersink->videosink.height;

    /* Display position and size */
    luapiconfig.layerConfig.screen_win.x = framebuffersink->video_rectangle.x;
    luapiconfig.layerConfig.screen_win.y = framebuffersink->video_rectangle.y;
    luapiconfig.layerConfig.screen_win.width = framebuffersink->video_rectangle.w;
    luapiconfig.layerConfig.screen_win.height = framebuffersink->video_rectangle.h;

    luapiconfig.layerConfig.alpha_mode = 0;
    luapiconfig.layerConfig.fb.pre_multiply = 0;
    luapiconfig.layerConfig.alpha_value = 0xff;
    luapiconfig.layerConfig.zorder = 3;
    luapiconfig.layerConfig.mode = DISP_LAYER_WORK_MODE_SCALER;
    luapiconfig.layerConfig.pipe = 0;
#endif

    if (DispSetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                1, &luapiconfig) < 0)
		return FALSE;

    gst_sunxifbsink_show_layer(sunxifbsink);

  return GST_FLOW_OK;
}

static GstFlowReturn
gst_sunxifbsink_show_overlay (GstFramebufferSink *framebuffersink,
    GstMemory *memory)
{
  GstSunxifbsink *sunxifbsink = GST_SUNXIFBSINK (framebuffersink);
  GstFbdevFramebufferSink *fbdevframebuffersink =
      GST_FBDEVFRAMEBUFFERSINK (framebuffersink);
  GstMapInfo mapinfo;
  guintptr framebuffer_offset, framebuffer_vir;
  GstFlowReturn res;
  struct SunxiMemOpsS* ops =  GetMemAdapterOpsS();

  gst_memory_map(memory, &mapinfo, GST_MAP_READ);
  memcpy(sunxifbsink->sBuffer, mapinfo.data, sizeof(OmxPrivateBuffer));
  gst_memory_unmap(memory, &mapinfo);

  framebuffer_offset = sunxifbsink->sBuffer->pAddrPhyY;
  framebuffer_vir = sunxifbsink->sBuffer->pAddrVirY;

 GST_LOG_OBJECT (sunxifbsink, "Show overlay called (offset = 0x%08lX)",
      framebuffer_offset);

  res = GST_FLOW_ERROR;
  if(GST_MEMORY_FLAG_IS_SET(memory, GST_MEMORY_FLAG_PHYSICALLY_CONTIGUOUS))
  {
		gst_sunxifbsink_show_memory_yuv_planar(framebuffersink,
			sunxifbsink->overlay_format,memory);
  }
  else
  {
	  if(framebuffersink->max_video_memory_property <= 0)
	  {
		framebuffer_offset+=fbdevframebuffersink->fixinfo.smem_start;
	  }
	  else
	  {
	    SunxiMemFlushCache(ops, framebuffer_vir, framebuffersink->video_info.size);
		//framebuffer_offset = (guintptr)SunxiMemGetPhysicAddressCpu(ops, framebuffer_vir);
	  }

	  if (sunxifbsink->overlay_format == GST_VIDEO_FORMAT_I420 ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_YV12 ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_Y444 ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_NV12 ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_NV21)
	    res =  gst_sunxifbsink_show_overlay_yuv_planar (framebuffersink,
	        framebuffer_offset, sunxifbsink->overlay_format);
	  else if (sunxifbsink->overlay_format == GST_VIDEO_FORMAT_YUY2 ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_UYVY ||
	      sunxifbsink->overlay_format == GST_VIDEO_FORMAT_AYUV)
	    res =  gst_sunxifbsink_show_overlay_yuv_packed (framebuffersink,
	        framebuffer_offset, sunxifbsink->overlay_format);
	  else if (sunxifbsink->overlay_format == GST_VIDEO_FORMAT_BGRx)
	    res = gst_sunxifbsink_show_overlay_bgrx32 (framebuffersink,
	        framebuffer_offset);
  }

  //gst_memory_unmap(memory, &mapinfo);
  return res;
}

static gboolean
gst_sunxifbsink_reserve_layer(GstSunxifbsink *sunxifbsink) {

    luapi_layer_config luapiconfig;
    unsigned int tmp = 0;
	int screen_w, screen_h;
	gchar s[256];

    screen_w = DispGetScrWidth(sunxifbsink->fd_disp, tmp);
    if(screen_w < 0){
        g_sprintf(s,"-->screen get win width error.errno(%d)", errno);
        GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink, s);
    }
    screen_h = DispGetScrHeight(sunxifbsink->fd_disp, tmp);
    if(screen_h < 0){
        g_sprintf(s,"-->screen get win height error.errno(%d)", errno);
        GST_SUNXIFBSINK_ERROR_OBJECT (sunxifbsink, s);
    }

	 g_sprintf(s,"-->reserver layer called (screen = %d x %d)",screen_w, screen_h);
	 GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, s);

    /* try to allocate a layer */
    memset(&luapiconfig, 0, sizeof(luapiconfig));
#ifdef __SUNXI_DISPLAY2__
    luapiconfig.layerConfig.info.screen_win.x = 0;
    luapiconfig.layerConfig.info.screen_win.y = 0;
    luapiconfig.layerConfig.info.screen_win.width = screen_w;
    luapiconfig.layerConfig.info.screen_win.height = screen_h;

	luapiconfig.layerConfig.info.fb.crop.x = 0;
    luapiconfig.layerConfig.info.fb.crop.y = 0;
    luapiconfig.layerConfig.info.fb.crop.width = (unsigned long long)screen_w << 32;
    luapiconfig.layerConfig.info.fb.crop.height = (unsigned long long)screen_h << 32;

	luapiconfig.layerConfig.enable = FALSE;
	luapiconfig.layerConfig.layer_id = sunxifbsink->layer_id;
	luapiconfig.layerConfig.channel = sunxifbsink->framebuffer_id;
	luapiconfig.layerConfig.info.mode= LAYER_MODE_BUFFER;
	luapiconfig.layerConfig.info.fb.flags= DISP_BF_NORMAL;
	luapiconfig.layerConfig.info.fb.scan= DISP_SCAN_PROGRESSIVE;
	luapiconfig.layerConfig.info.fb.format = DISP_FORMAT_ARGB_8888;
	luapiconfig.layerConfig.info.zorder = 11;
	luapiconfig.layerConfig.info.alpha_mode = 1;
	luapiconfig.layerConfig.info.alpha_value = 0xff;
#else
	luapiconfig.layerConfig.fb.src_win.x      = 0;
	luapiconfig.layerConfig.fb.src_win.y      = 0;
	luapiconfig.layerConfig.fb.src_win.width  = 1;
	luapiconfig.layerConfig.fb.src_win.height = 1;
	luapiconfig.layerConfig.fb.size.width = screen_w;
	luapiconfig.layerConfig.fb.size.height = screen_h;
	luapiconfig.layerConfig.screen_win.x = 0;
	luapiconfig.layerConfig.screen_win.y = 0;
	luapiconfig.layerConfig.screen_win.width = screen_w;
	luapiconfig.layerConfig.screen_win.height = screen_h;
	luapiconfig.layerConfig.alpha_mode = 0;
	luapiconfig.layerConfig.fb.pre_multiply = 0;
	luapiconfig.layerConfig.alpha_value = 0xff;
	luapiconfig.layerConfig.zorder = 3;
	luapiconfig.layerConfig.mode = DISP_LAYER_WORK_MODE_SCALER;
	luapiconfig.layerConfig.pipe = 0;
#endif

    if (DispSetLayerConfig(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
		                                1, &luapiconfig) < 0)
		return FALSE;

    sunxifbsink->layer_has_scaler = TRUE;
  return TRUE;
}

static void
gst_sunxifbsink_release_layer(GstSunxifbsink *sunxifbsink) {

	GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->sunxifbsink_release_layer");

    if(sunxifbsink->layer_is_visible){
        DispSetLayerEnable(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
          sunxifbsink->framebuffer_id, 1, 0);
        sunxifbsink->layer_is_visible = FALSE;
    }
    sunxifbsink->layer_id = -1;
    sunxifbsink->layer_has_scaler = 0;
}

static gboolean gst_sunxifbsink_show_layer(GstSunxifbsink *sunxifbsink) {

  gchar s[256];

  if (sunxifbsink->layer_is_visible)
    return TRUE;

  if (sunxifbsink->layer_id < 0)
    return FALSE;

  g_sprintf(s,"-->show_layer id (%d)",sunxifbsink->layer_id);
  GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, s);

  if (DispSetLayerEnable(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
          sunxifbsink->framebuffer_id, 1, 1)){
        return FALSE;
    }

  sunxifbsink->layer_is_visible = TRUE;
  return TRUE;
}

static void gst_sunxifbsink_hide_layer(GstSunxifbsink *sunxifbsink) {

  GST_SUNXIFBSINK_MESSAGE_OBJECT (sunxifbsink, "-->sunxifbsink_hide_layer");

  if (!sunxifbsink->layer_is_visible)
    return;

  if (DispSetLayerEnable(sunxifbsink->fd_disp, sunxifbsink->framebuffer_id, sunxifbsink->layer_id,
          sunxifbsink->framebuffer_id, 1, 0)){
        return;
    }

  sunxifbsink->layer_is_visible = FALSE;
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  /* Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  return gst_element_register (plugin, "sunxifbsink", GST_RANK_SECONDARY,
      GST_TYPE_SUNXIFBSINK);
}

/* these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
#ifndef VERSION
#define VERSION "0.1"
#endif
#ifndef PACKAGE
#define PACKAGE "gstsunxifbsink"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "gstreamer1.0-sunxi-plugins"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "https://github.com/hglm"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    sunxifbsink,
    "Accelerated console framebuffer video sink for sunxi-based devices",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)
