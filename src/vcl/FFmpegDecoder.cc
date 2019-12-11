#include "vcl/FFmpegDecoder.h"

using namespace VCL;

static enum AVPixelFormat ghw_pix_fmt;

static enum AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                        const enum AVPixelFormat *pix_fmts)
{
    const enum AVPixelFormat *p;

    for (p = pix_fmts; *p != -1; p++) {
        if (*p == ghw_pix_fmt)
            return *p;
    }

    fprintf(stderr, "Failed to get HW surface format.\n");
    return AV_PIX_FMT_NONE;
}

FFmpegDecoder::FFmpegDecoder()
{
}

FFmpegDecoder::FFmpegDecoder(const std::string& input_file)
{
    init(input_file,"");
}

FFmpegDecoder::~FFmpegDecoder()
{
    close();
}

void FFmpegDecoder::close()
{
    if (OUTHDL)
    {
        fclose(OUTHDL);
        OUTHDL = NULL;
    }

    if (pWriterBuf) av_freep(&pWriterBuf);
    if (codec_ctx) avcodec_free_context(&(codec_ctx));
    if (fmt_ctx) avformat_close_input(&(fmt_ctx));
    if (hw_device_ctx) av_buffer_unref(&(hw_device_ctx));
}

int FFmpegDecoder::init(const std::string& input_file, const std::string& output_file)
{
    const char* device="cpu";

    int ret;

    /* open the input file */
    if (avformat_open_input(&fmt_ctx, input_file.c_str(), NULL, NULL) != 0) {
        fprintf(stderr, "Cannot open input file '%s'\n", input_file.c_str());
        return -1;
    }
    infile = input_file;

    if (!outfile.empty())
    {
        /* open the output file */
        outfile = output_file;
        OUTHDL = fopen(output_file.c_str(), "w+");

        if (!OUTHDL) {
            fprintf(stderr, "Failed to output file: %s\n", output_file.c_str());
            return -1;
        }
    }

    // Check input device, CPU device will be chosen if not found
    dev_type = av_hwdevice_find_type_by_name(device);
//    fprintf(stdout, "\n### Using device: %s ###\n\n", !(dev_type)? "cpu" : av_hwdevice_get_type_name(dev_type));

    if (avformat_find_stream_info(fmt_ctx, NULL) < 0) {
        fprintf(stderr, "Cannot find input stream information.\n");
        return -1;
    }

    /* find the video stream information */
    vstrm_idx = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    if (vstrm_idx < 0) {
        fprintf(stderr, "Cannot find a video stream in the input file\n");
        return -1;
    }
    vstrm = fmt_ctx->streams[vstrm_idx];

    if (dev_type != AV_HWDEVICE_TYPE_NONE)
    {
        for (uint8_t i = 0;; i++) {
            const AVCodecHWConfig *config = avcodec_get_hw_config(codec, i);
            if (!config) {
                fprintf(stderr, "Decoder %s does not support device type %s.\n",
                        codec->name, av_hwdevice_get_type_name(dev_type));
                return -1;
            }
            if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
                config->device_type == dev_type) {
                ghw_pix_fmt = config->pix_fmt;
                break;
            }
        }
    }

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        fprintf(stderr, "Failed to allocate codec\n");
        return AVERROR(EINVAL);
    }

    ret = avcodec_parameters_to_context(codec_ctx, vstrm->codecpar);
    if (ret < 0) {
        fprintf(stderr, "Failed to copy codec parameters to codec context\n");
        return ret;
    }

    if (dev_type != AV_HWDEVICE_TYPE_NONE)
    {
        codec_ctx->get_format  = get_hw_format;

        if (hw_decoder_init() < 0)
            return -1;
    }

    if ((ret = avcodec_open2(codec_ctx, codec, NULL)) < 0) {
        fprintf(stderr, "Failed to open codec for stream #%u\n", vstrm_idx);
        return -1;
    }

    av_dump_format(fmt_ctx, 0, input_file.c_str(), 0);

    isInit=true;
    return 0;
}

int FFmpegDecoder::hw_decoder_init()
{
    int err = 0;

    if ((err = av_hwdevice_ctx_create(&hw_device_ctx, dev_type, NULL, NULL, 0)) < 0) {
        fprintf(stderr, "Failed to create specified HW device.\n");
        return err;
    }
    codec_ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);

    return err;
}

int FFmpegDecoder::seek(int64_t timestamp)
{
    return av_seek_frame(fmt_ctx, vstrm_idx, timestamp, AVSEEK_FLAG_ANY);
}

std::vector<KeyFrame> FFmpegDecoder::get_keyframe_list()
{
    if (!isInit)
        throw VCLException(FFmpegNotInit, "FFmpeg Decoder is not initialized");

    int      dispId=0;
    AVFrame* pFrame = NULL;
    KeyFrame keyframe;

    std::vector<KeyFrame> keyframe_list;

    while (pFrame = get_one_frame())
    {
        if (pFrame->key_frame)
        {
            float ts_sec = (float)pFrame->pkt_dts*
                           (float)vstrm->time_base.num/
                           (float)vstrm->time_base.den;

            float period = (float)vstrm->avg_frame_rate.den/
                           (float)vstrm->avg_frame_rate.num;

            keyframe.pkt_pos        = pFrame->pkt_pos;
            keyframe.pkt_ts         = pFrame->best_effort_timestamp;
            keyframe.time_base      = vstrm->time_base;
            keyframe.avg_frame_rate = vstrm->avg_frame_rate;
            keyframe.derivedId      = (int)((float)ts_sec/(float)period + 0.5);

            keyframe_list.push_back(keyframe);

//            printf("[get_keyframe_attr] DisId:%4d | pos: %d | dts: %d | pFrame->pts: %d | ts: %f | period: %f | derivedID: %0.5f\n",
//                   dispId,
//                   keyframe.pkt_pos,
//                   keyframe.pkt_ts,
//                   pFrame->best_effort_timestamp,
//                   ts_sec,
//                   period,
//                   ts_sec/period
//                   );
//            cout<<flush;
        }

        av_frame_free(&pFrame);
        pFrame = NULL;
        dispId++;
    }

    return keyframe_list;
}

AVFrame* FFmpegDecoder::get_one_frame()
{
    int ret=0;

    /* actual decoding and dump the raw data */
    while (ret >= 0)
    {      
        if ((ret = av_read_frame(fmt_ctx, &avpkt)) < 0)
            break;

        if (vstrm_idx == avpkt.stream_index)
        {
            AVFrame *frame = NULL;
            AVFrame *sw_frame = NULL;
            AVFrame *tmp_frame = NULL;
            uint8_t *buffer = NULL;
            int size;
            int ret = 0;

            ret = avcodec_send_packet(codec_ctx, &avpkt);
            if (ret < 0)
            {
                fprintf(stderr, "Error during decoding\n");
                return NULL;
            }

            while (1)
            {
                if (!(frame = av_frame_alloc()) || !(sw_frame = av_frame_alloc()))
                {
                    fprintf(stderr, "Can not alloc frame\n");
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }

                ret = avcodec_receive_frame(codec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                {
                    av_frame_free(&frame);
                    av_frame_free(&sw_frame);
                    break;
                }
                else if (ret < 0) {
                    fprintf(stderr, "Error while decoding\n");
                    goto fail;
                }

                if (frame->format == AV_PIX_FMT_VAAPI_VLD)
                {
                    /* retrieve data from GPU to CPU */
                    if ((ret = av_hwframe_transfer_data(sw_frame, frame, 0)) < 0)
                    {
                        fprintf(stderr, "Error transferring the data to system memory\n");
                        goto fail;
                    }
                    tmp_frame = sw_frame;
                }
                else
                {
                    tmp_frame = frame;
                }

                // frame is a hwframe if vaapi device is used.
                // frame is freed as it has been copied to sw_frame
                if (frame->format == AV_PIX_FMT_VAAPI_VLD)
                    av_frame_free(&frame);

                // return decoded frame
                return tmp_frame;

                fail:
                    av_frame_free(&frame);
                    av_frame_free(&sw_frame);
                }
            }

        av_packet_unref(&avpkt);
    }

    /* flush the decoder */
    avpkt.data = NULL;
    avpkt.size = 0;
    av_packet_unref(&(avpkt));

    return NULL;
}

int FFmpegDecoder::write_frame(AVFrame* avframe)
{
    int ret;
    if (OUTHDL) {
        if (!pWriterBuf)
        {
            WriterBufSize = av_image_get_buffer_size((AVPixelFormat)avframe->format, avframe->width,
                                            avframe->height, 1);
            pWriterBuf = (uint8_t*)av_malloc(WriterBufSize);
            if (!pWriterBuf) {
                fprintf(stderr, "Can not alloc WriterBuf\n");
                ret = AVERROR(ENOMEM);
            }

        }

        ret = av_image_copy_to_buffer(pWriterBuf, WriterBufSize,
                                      (const uint8_t * const *)avframe->data,
                                      (const int *)avframe->linesize,
                                      (AVPixelFormat)avframe->format,
                                      avframe->width, avframe->height, 1);

        if (ret < 0) {
            fprintf(stderr, "Can not copy image to WriterBuf\n");
        }

        if ((ret = fwrite(pWriterBuf, 1, WriterBufSize, OUTHDL)) < 0) {
            fprintf(stderr, "Failed to dump raw data from WriterBuf.\n");
        }
        return 0;
    }
    else
    {
        fprintf(stderr, "Output handle has not been set up.\n");
        return -1;
    }
}

void FFmpegDecoder::plot(Mat &ocv_frame, bool default_winsize)
{
    // = Plot Frame =
    string window_label = "FFmpegDecoder";

    namedWindow(window_label, WINDOW_KEEPRATIO);
    // active low
    if (!default_winsize) resizeWindow(window_label, 1080, 720);
    imshow(window_label, ocv_frame);
    waitKey(1);
    // = EO Plot Frame =
}

void FFmpegDecoder::avframeToMat(const AVFrame * frame, Mat& image)
{
    int width = frame->width;
    int height = frame->height;
    int tmp_fmt = frame->format;

    // workaround to avoid warning of deprecated format
    switch (frame->format)
    {
        case AV_PIX_FMT_YUVJ420P:
          tmp_fmt = AV_PIX_FMT_YUV420P;
          break;
        case AV_PIX_FMT_YUVJ422P:
          tmp_fmt = AV_PIX_FMT_YUV422P;
          break;
        case AV_PIX_FMT_YUVJ444P:
          tmp_fmt = AV_PIX_FMT_YUV444P;
          break;
        case AV_PIX_FMT_YUVJ440P:
          tmp_fmt = AV_PIX_FMT_YUV440P;
          break;
        default:
        {}
    }

    // Allocate the opencv mat and store its stride in a 1-element array
    if (image.rows != height || image.cols != width || image.type() != CV_8UC3) image = Mat(height, width, CV_8UC3);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the colour format and write directly to the opencv matrix
//    SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat) tmp_fmt, width, height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
}

