/**
 * @file   Video.cc
 *
 * @section LICENSE
 *
 * The MIT License
 *
 * @copyright Copyright (c) 2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <fstream>
#include <algorithm>

#include <iostream> // cout

#include "vcl/Video.h"

using namespace VCL;

    /*  *********************** */
    /*        CONSTRUCTORS      */
    /*  *********************** */

Video::Video() :
    _size({.width = 0, .height = 0, .frame_count = 0}),
    _fps(0),
    _video_id(""),
    _flag_stored(true),
    _codec(Video::Codec::NOCODEC)
{
}

Video::Video(const std::string& video_id) :
    Video()
{
    _video_id = video_id;
    populate_video_params();
}

Video::Video(void* buffer, long size) :
    Video()
{
    std::string uname = create_unique("/tmp/", "vclvideoblob");
    std::ofstream outfile(uname, std::ofstream::binary);

    if (outfile.is_open()) {
        outfile.write((char*)buffer, size);
        outfile.close();
    }
    else
        throw VCLException(OpenFailed, "Cannot create temporary file");

    _video_id = uname;
    populate_video_params();
}

Video::Video(const Video &video)
{
    _video_id = video._video_id;

    _size = video._size;

    _fps   = video._fps;
    _codec = video._codec;

    _video_id = video.get_video_id();
    _codec    = video.get_codec();

    _flag_stored = video._flag_stored;

    _frames     = video._frames;
    _operations = video._operations;

    for (const auto& op : video._operations)
        _operations.push_back(op);

    for (const auto& foi: video._foi)
        _foi.push_back(foi);
}

Video& Video::operator=(Video vid)
{
    swap(vid);
    return *this;
}

Video::~Video()
{
    _operations.clear();
}

    /*  *********************** */
    /*        GET FUNCTIONS     */
    /*  *********************** */

std::string Video::get_video_id() const
{
    return _video_id;
}

Video::Codec Video::get_codec() const
{
    return _codec;
}

cv::Mat Video::get_frame(int frame_number)
{
    if( _frames.empty())
        perform_operations();

    if (_frames.find(frame_number) == _frames.end())
    {
//          throw VCLException(OutOfBounds, "Frame requested is out of bounds");
        cout << "Warning: Frame not found, return black frame for id " << frame_number << endl;
        Mat black_img(_size.width, _size.height, CV_8UC3, Scalar(0,0,0));
        return black_img;
    }
    return _frames.at(frame_number).get_cvmat();
}

long Video::get_frame_count()
{
    perform_operations();
    return _size.frame_count;
}

float Video::get_fps()
{
    return _fps;
}

cv::Size Video::get_frame_size()
{
    perform_operations();
    cv::Size dims((int) _size.width,
                  (int) _size.height);
    return dims;
}

Video::VideoSize Video::get_size()
{
    perform_operations();
    return _size;
}

std::vector<unsigned char> Video::get_encoded()
{
    if (_flag_stored == false)
        throw VCLException(ObjectEmpty, "Object not written");

    std::ifstream ifile(_video_id, std::ifstream::in);
    ifile.seekg(0, std::ios::end);
    size_t encoded_size = (long)ifile.tellg();
    ifile.seekg(0, std::ios::beg);

    std::vector<unsigned char> encoded(encoded_size);

    ifile.read((char*)encoded.data(), encoded_size);
    ifile.close();

    return encoded;
}

const KeyFrameList& Video::get_key_frame_list()
{
    FFmpegDecoder dec(_video_id);
    _key_frame_list = dec.get_keyframe_list();
    return _key_frame_list;
}

    /*  *********************** */
    /*        SET FUNCTIONS     */
    /*  *********************** */

void Video::set_video_id(const std::string &video_id)
{
    _video_id = video_id;
}

void Video::set_codec(Video::Codec codec)
{
    _codec = codec;
}

void Video::set_dimensions(const cv::Size& dimensions)
{
    _size.height = dimensions.height;
    _size.width  = dimensions.width;
}

void Video::set_foi(const std::vector<unsigned int> &foi)
{
    _foi.clear();
    if (!foi.empty())
    {
        _foi = foi;
        std::sort(_foi.begin(),_foi.end());
    }
    else
    {
        throw VCLException(ObjectEmpty, "Empty FOI vector");
    }
}

void Video::populate_video_params()
{
    cv::VideoCapture inputVideo(_video_id);

    _fps = static_cast<float>(inputVideo.get(cv::CAP_PROP_FPS));
    _size.frame_count  = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_COUNT));
    _size.width        = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_WIDTH));
    _size.height       = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT));

    // Get Codec Type- Int form
    int ex = static_cast<int>(inputVideo.get(cv::CAP_PROP_FOURCC));
    char fourcc[] = {(char)((ex & 0XFF)),
                     (char)((ex & 0XFF00) >> 8),
                     (char)((ex & 0XFF0000) >> 16),
                     (char)((ex & 0XFF000000) >> 24),
                     0};

    // TODO - Following routine should be part of utilities
    // Duplicated in Videocommands string_to_codec and READ operation read_codec
    std::string codec(fourcc);
    std::transform(codec.begin(), codec.end(), codec.begin(), ::tolower);

    if (codec == "mjpg")
        _codec = Codec::MJPG;
    else if (codec == "xvid")
        _codec = Codec::XVID;
    else if (codec == "u263")
        _codec = Codec::H263;
    else if (codec == "avc1" || codec == "x264")
        _codec = Codec::H264;
    else
        throw VCLException(UnsupportedFormat, codec + " is not supported");

    inputVideo.release();

    init_foi();
}

    /*  *********************** */
    /*       UTILITIES          */
    /*  *********************** */

bool Video::is_read(void)
{
    return (_frames.size() > 0);
}

void Video::perform_operations()
{
    try
    {
        // At this point, there are three different potential callees:
        //
        // - An object is instantiated through the default constructor with
        //   no name: an exception is thrown as no operations can be applied.
        //
        // - An object is instantiated through one-arg string constructor,
        //   but has no operations set explicitely (i.e. when calling
        //   get_frame_count()): a 'read' operation is pushed to the head of
        //   the queue.
        //
        // - An object is instantiated through any of the non-default
        //   constructors, and has pushed operations explicitely: a 'read'
        //   operation is pushed to the head of the queue.
        if ((_operations.empty() || _operations.front()->get_type() != READ)
            && !is_read()) {
            if (_video_id.empty())
                throw VCLException(OpenFailed, "video_id is not initialized");
            _operations.push_front(std::make_shared<Read>());
        }
        for (const auto& op : _operations) {
            if ( op == NULL )
                throw VCLException(ObjectEmpty, "Nothing to be done");
            (*op)(this);
        }
    } catch( cv::Exception& e ) {
        throw VCLException(OpenCVError, e.what());
    }

    _operations.clear();
}

void Video::swap(Video& rhs) noexcept
{
    using std::swap;

    swap(_video_id, rhs._video_id);
    swap(_flag_stored, rhs._flag_stored);
    swap(_frames, rhs._frames);
    swap(_size, rhs._size);
    swap(_fps, rhs._fps);
    swap(_codec, rhs._codec);
    swap(_operations, rhs._operations);
    swap(_foi, rhs._foi);
}

    /*  *********************** */
    /*   VIDEO INTERACTION      */
    /*  *********************** */

void Video::resize(int width, int height)
{
    _flag_stored = false;
    _operations.push_back(std::make_shared<Resize>(cv::Size(width, height)));
}

void Video::interval(Video::Unit u, int start, int stop, int step)
{
    _flag_stored = false;
    _operations.push_back(std::make_shared<Interval>(u, start, stop, step));
}

void Video::crop(const Rectangle &rect)
{
    _flag_stored = false;
    _operations.push_back(std::make_shared<Crop>(rect));
}

void Video::threshold(int value)
{
    _flag_stored = false;
    _operations.push_back(std::make_shared<Threshold>(value));
}

void Video::store(const std::string &video_id, Video::Codec video_codec)
{
    if ((video_codec == _codec) && (_operations.size() == 0) &&
            (_foi.size()==_size.frame_count)) {
        //We copy/move no change in codec and no operation and FOI is all frames
        moveto(video_id);
    } else {
        // out_name cannot be assigned to _video_id here as the read operation
        // may be pending and the input file name is needed for the read.
        _operations.push_back(std::make_shared<Write>(video_id, video_codec));
        perform_operations();
    }
}

void Video::store()
{
    if (_codec == NOCODEC || _video_id.empty()) {
        throw VCLException(ObjectEmpty, "Cannot write video without codec"
                                        "or ID");
    }
    store(_video_id, _codec);
}

void Video::moveto(const std::string &video_path)
{
    if (!_video_id.empty())
    {
        std::filebuf infile, outfile;
        infile.open(_video_id, std::ios::in | std::ios::binary);
        outfile.open(video_path, std::ios::out | std::ios::binary);

        std::copy(std::istreambuf_iterator<char>(&infile), {},
                  std::ostreambuf_iterator<char>(&outfile));
    }
    else
        throw VCLException(ObjectEmpty, "video_input path not found");
}

void Video::delete_video()
{
    if (exists(_video_id)) {
        std::remove(_video_id.c_str());
    }
}

void Video::init_foi()
{
    _foi.clear();
    for (unsigned int i=0; i<_size.frame_count;i++)
        _foi.push_back(i);
}

    /*  *********************** */
    /*       READ OPERATION    */
    /*  *********************** */

Video::Codec Video::Read::read_codec(char* fourcc)
{
    std::string codec(fourcc);
    std::transform(codec.begin(), codec.end(), codec.begin(), ::tolower);

    if (codec == "mjpg")
        return Codec::MJPG;
    else if (codec == "xvid")
        return Codec::XVID;
    else if (codec == "u263")
        return Codec::H263;
    else if (codec == "avc1" || codec == "x264")
        return Codec::H264;
    else
        throw VCLException(UnsupportedFormat, codec + " is not supported");
}

void Video::Read::operator()(Video *video)
{
    cv::VideoCapture inputVideo(video->_video_id);

    video->_fps = static_cast<float>(inputVideo.get(cv::CAP_PROP_FPS));
    video->_size.frame_count  = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_COUNT));
    video->_size.width        = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_WIDTH));
    video->_size.height       = static_cast<int>(
                                inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT));

    // Get Codec Type- Int form
    int ex = static_cast<int>(inputVideo.get(cv::CAP_PROP_FOURCC));
    char fourcc[] = {(char)((ex & 0XFF)),
                     (char)((ex & 0XFF00) >> 8),
                     (char)((ex & 0XFF0000) >> 16),
                     (char)((ex & 0XFF000000) >> 24),
                     0};

    video->_codec = read_codec(fourcc);

    video->_frames.clear();

    FFmpegDecoder dec(video->_video_id);
    AVFrame* pFrame = NULL;
    cv::Mat mat_frame;
    int frameId;

    float timebase = (float)dec.vstrm->time_base.num/
                     (float)dec.vstrm->time_base.den;

    float period = (float)dec.vstrm->avg_frame_rate.den/
                   (float)dec.vstrm->avg_frame_rate.num;

    // This a scaler to convert timestamp counter to frame Id
    float scaler = timebase/period;

    FFmpegDecoder tmpDec(video->_video_id); // to be deleted
    KeyFrameList kfList=tmpDec.get_keyframe_list();
    tmpDec.close(); // to be deleted

    KeyFrame kf_start=kfList[0];

    // finding the kf to begin decode
    // Assuming kfList in ascending order of keyframe
    for (auto& kf: kfList)
    {
        if (kf.derivedId > video->_foi.front())
            break;
        kf_start = kf;
    }

    dec.seek(kf_start.pkt_ts);

    while (pFrame = dec.get_one_frame())
    {
        frameId = (int)( ((float)pFrame->best_effort_timestamp * scaler) + 0.5 );
        dec.avframeToMat(pFrame, mat_frame);
        video->_frames.insert(make_pair(frameId, VCL::Image(mat_frame, true)));
        av_frame_free(&pFrame);
        pFrame = NULL;

        if (frameId == video->_foi.back())
            break;
    }

    dec.close();
    inputVideo.release();
}

    /*  *********************** */
    /*       WRITE OPERATION    */
    /*  *********************** */

int Video::Write::get_fourcc()
{
    switch(_codec)
    {
        case Codec::MJPG:
            return cv::VideoWriter::fourcc('M','J','P','G');
        case Codec::XVID:
            return cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        case Codec::H263:
            return cv::VideoWriter::fourcc('U', '2', '6', '3');
        case Codec::H264:
            return cv::VideoWriter::fourcc('X', '2', '6', '4');
        case Codec::AVC1:
            return cv::VideoWriter::fourcc('A', 'V', 'C', '1');
        default:
            throw VCLException(UnsupportedFormat, std::to_string((int)_codec) +
                               " is not a valid format");
    }
}

void Video::Write::operator()(Video *video)
{
    cv::VideoWriter outputVideo(
                    _outname,
                    get_fourcc(),
                    video->_fps,
                    cv::Size(video->_size.width, video->_size.height));

    if (!outputVideo.isOpened()) {
        throw VCLException(OpenFailed,
                "Could not open the output video for write");
    }

    for (auto& foi : video->_foi)
    {
        outputVideo << video->get_frame(foi);
    }

    outputVideo.release();

    video->_video_id = _outname;
    video->_codec    = _codec;
    video->_flag_stored = true;
}

    /*  *********************** */
    /*       RESIZE OPERATION   */
    /*  *********************** */

void Video::Resize::operator()(Video *video)
{
    for (auto& frame : video->_frames) {
        // VCL::Image expect the params (h,w) (contrary to openCV convention)
        frame.second.resize(_size.height, _size.width);
    }

    video->_size.width  = _size.width;
    video->_size.height = _size.height;
}

    /*  *********************** */
    /*       CROP OPERATION     */
    /*  *********************** */

void Video::Crop::operator()(Video *video)
{
    for (auto& frame : video->_frames) {
        frame.second.crop(_rect);
    }

    video->_size.width  = _rect.width;
    video->_size.height = _rect.height;
}

    /*  *********************** */
    /*    THRESHOLD OPERATION   */
    /*  *********************** */

void Video::Threshold::operator()(Video *video)
{
    for (auto& frame : video->_frames) {
        frame.second.threshold(_threshold);
    }
}

    /*  *********************** */
    /*   INTERVAL Operation     */
    /*  *********************** */

void Video::Interval::operator()(Video *video)
{
    if (_u != Video::Unit::FRAMES)
        throw VCLException(UnsupportedOperation,
                "Only Unit::FRAMES supported for interval operation");

    std::map<int, VCL::Image>& frames = video->_frames;
    unsigned nframes = frames.size();

    if (_start >= nframes)
        throw VCLException(SizeMismatch,
                "Start Frame cannot be greater than number of frames");

    if ((_stop-_step) >= nframes)
        throw VCLException(SizeMismatch,
                "End Frame cannot be greater than number of frames");

    std::vector<unsigned int> foi;
    for (unsigned int i = _start; i < _stop; i += _step) {
        foi.push_back(i);
    }
    video->set_foi(foi);

    video->_size.frame_count = foi.size();

    // Do we want to remove the unwanted frame from cache?

    //TODO: duration preservation doesnt work
//    video->_fps /= _step;
//    video->_size.frame_count = interval_vector.size();
//
}
