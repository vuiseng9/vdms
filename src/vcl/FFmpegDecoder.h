#ifndef _FFMPEGDECODER_HPP_
#define _FFMPEGDECODER_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
#include <libavutil/avassert.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "vcl/Exception.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace VCL {

    struct KeyFrame {
        int derivedId;
        int64_t pkt_ts;
        int64_t pkt_pos;
        AVRational time_base;
        AVRational avg_frame_rate;
    };

    typedef std::vector<KeyFrame> KeyFrameList;

    class FFmpegDecoder
    {
        int _keyframe_idx=-1;
        bool isInit=false;
    public:
        enum AVHWDeviceType dev_type;
        enum AVPixelFormat hw_pix_fmt;

        AVCodec*         codec = NULL;
        AVCodecContext*  codec_ctx = NULL;
        AVPacket         avpkt;
        AVFormatContext* fmt_ctx = NULL;
        AVStream*        vstrm = NULL;
        int              vstrm_idx;
        AVBufferRef*     hw_device_ctx = NULL;

        string       infile;
        string       outfile;
        FILE*        OUTHDL = NULL;
        uint8_t*     pWriterBuf = NULL;
        int          WriterBufSize = 0;

        FFmpegDecoder();
        FFmpegDecoder(const std::string& input_file);
        ~FFmpegDecoder();

        int init(const std::string& input_file, const std::string& output_file);
        int hw_decoder_init();
        int seek(int64_t timestamp);

        AVFrame* get_one_frame();
        void     avframeToMat(const AVFrame * frame, Mat& image);
        std::vector<KeyFrame> get_keyframe_list();
        int      write_frame(AVFrame* avframe);
        void     plot(Mat &ocv_frame, bool default_winsize);

        void close();
    };
}
#endif /* _FFMPEGDECODER_HPP_ */
