/**
 * @file   VideoCommand.cc
 *
 * @section LICENSE
 *
 * The MIT License
 *
 * @copyright Copyright (c) 2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <iostream>
#include <fstream>

#include "ImageCommand.h" // for enqueue_operations of Image type
#include "VideoCommand.h"
#include "VDMSConfig.h"
#include "defines.h"
#include <random>

using namespace VDMS;

void print_json(std::string label, Json::Value& json)
{
    Json::StreamWriterBuilder builder;
    builder["indentation"]="  ";
    cout << "[JSON Printer] " << label << endl
         << Json::writeString(builder, json) << endl << std::flush;
}

VideoCommand::VideoCommand(const std::string &cmd_name):
    RSCommand(cmd_name)
{
}

void VideoCommand::enqueue_operations(VCL::Video& video, const Json::Value& ops)
{
    // Correct operation type and parameters are guaranteed at this point
    for (auto& op : ops) {
        const std::string& type = get_value<std::string>(op, "type");
         std::string unit ;
        if (type == "threshold") {
            video.threshold(get_value<int>(op, "value"));

        }
        else if (type == "interval") {

            video.interval(
                VCL::Video::FRAMES,
                get_value<int>(op, "start"),
                get_value<int>(op, "stop"),
                get_value<int>(op, "step"));

        }
        else if (type == "resize") {
             video.resize(get_value<int>(op, "height"),
                          get_value<int>(op, "width") );

        }
        else if (type == "crop") {
            video.crop(VCL::Rectangle (
                        get_value<int>(op, "x"),
                        get_value<int>(op, "y"),
                        get_value<int>(op, "width"),
                        get_value<int>(op, "height") ));
        }
        else {
            throw ExceptionCommand(ImageError, "Operation not defined");
        }
    }
}

VCL::Video::Codec VideoCommand::string_to_codec(const std::string& codec)
{
    if (codec == "h263") {
        return VCL::Video::Codec::H263;
    }
    else if (codec == "xvid") {
        return VCL::Video::Codec::XVID;
    }
    else if (codec == "h264") {
        return VCL::Video::Codec::H264;
    }

    return VCL::Video::Codec::NOCODEC;
}

Json::Value VideoCommand::check_responses(Json::Value& responses)
{
    if (responses.size() != 1) {
        Json::Value return_error;
        return_error["status"]  = RSCommand::Error;
        return_error["info"] = "PMGD Response Bad Size";
        return return_error;
    }

    Json::Value& response = responses[0];

    if (response["status"] != 0) {
        response["status"]  = RSCommand::Error;
        // Uses PMGD info error.
        return response;
    }

    return response;
}

// TODO: temporary overload to check responses according the user threshold
Json::Value VideoCommand::check_responses(Json::Value& responses, uint n_response)
{
    if (responses.size() != n_response) {
        Json::Value return_error;
        return_error["status"]  = RSCommand::Error;
        return_error["info"] = "PMGD Response Bad Size";
        return return_error;
    }

    Json::Value& response = responses[0];

    if (response["status"] != 0) {
        response["status"]  = RSCommand::Error;
        // Uses PMGD info error.
        return response;
    }

    return response;
}

//========= AddVideo definitions =========

AddVideo::AddVideo() : VideoCommand("AddVideo")
{
    _storage_video = VDMSConfig::instance()->get_path_videos();
}

int AddVideo::construct_protobuf(
    PMGDQuery& query,
    const Json::Value& jsoncmd,
    const std::string& blob,
    int grp_id,
    Json::Value& error)
{
    const Json::Value& cmd = jsoncmd[_cmd_name];

    int node_ref = get_value<int>(cmd, "_ref",
                                  query.get_available_reference());

    const std::string from_server_file = get_value<std::string>(cmd,
                                                        "from_server_file", "");

    VCL::Video video; // Video obj for input video

    if (from_server_file.empty())
    {
        // blob attacted to query is written to /tmp input video path
        video = VCL::Video((void*)blob.data(), blob.size());
    }
    else
    {
        video = VCL::Video(from_server_file);
    }

    if (cmd.isMember("operations")) {
        enqueue_operations(video, cmd["operations"]);
    }

    // The container and codec are checked by the schema.
    // We default to mp4 and h264, if not specified
    const std::string& container =
                            get_value<std::string>(cmd, "container", "mp4");
    const std::string& codec = get_value<std::string>(cmd, "codec", "h264");

    const std::string& file_name =
                            VCL::create_unique(_storage_video, container);

    VCL::Video::Codec vcl_codec = string_to_codec(codec);
    video.store(file_name, vcl_codec);

    // Modifiyng the existing properties that the user gives
    // is a good option to make the AddNode more simple.
    // This is not ideal since we are manupulating with user's
    // input, but for now it is an acceptable solution.
    Json::Value props = get_value<Json::Value>(cmd, "properties");
    props[VDMS_VID_PATH_PROP] = file_name;

    // add a label property to video entity
    string vid_label;
    if (!props.isMember("name"))
        vid_label = file_name;
    else
        vid_label = props["name"].asString();
    props[VDMS_VID_LABEL_PROP]=vid_label;

    // Add Video node
    query.AddNode(node_ref, VDMS_VID_TAG, props, Json::Value());

    // For now, keyframe extraction is only enabled for video with AVC encoding.
    // It is important to place this routine to ensure
    // final stored (could be transcoded) video is extracted.
    if ((VCL::Video::AVC1 == vcl_codec) || (VCL::Video::H264 == vcl_codec))
    {
        VCL::KeyFrameList keyframe_list;
        VCL::Video stored_video = VCL::Video(file_name);
        keyframe_list = stored_video.get_key_frame_list();

        // Add key-frames (if extracted) as nodes connected to the video
        for (const auto &keyframe : keyframe_list) {
            Json::Value keyframe_props;
            keyframe_props[VDMS_KF_IDX_PROP]     = static_cast<Json::Int>(keyframe.derivedId);
            keyframe_props[VDMS_KF_PKT_POS_PROP] = static_cast<Json::Int64>(keyframe.pkt_pos);
            keyframe_props[VDMS_KF_PKT_TS_PROP]  = static_cast<Json::Int64>(keyframe.pkt_ts);
            keyframe_props[VDMS_VID_LABEL_PROP]  = vid_label;

            int frame_ref = query.get_available_reference();
            query.AddNode(frame_ref, VDMS_KF_TAG, keyframe_props,
                          Json::Value());
            query.AddEdge(-1, node_ref, frame_ref, VDMS_KF_EDGE,
                          Json::Value());
        }
    }

    // In case we need to cleanup the query
    error["video_added"] = file_name;

    if (cmd.isMember("link")) {
        add_link(query, cmd["link"], node_ref, VDMS_VID_EDGE);
    }

    return 0;
}

Json::Value AddVideo::construct_responses(
    Json::Value& response,
    const Json::Value& json,
    protobufs::queryMessage &query_res,
    const std::string& blob)
{
    Json::Value ret;
    ret[_cmd_name] = RSCommand::check_responses(response);

    return ret;
}

bool AddVideo::need_blob(const Json::Value& cmd)
{
    const Json::Value& add_video_cmd = cmd[_cmd_name];
    return !(add_video_cmd.isMember("from_server_file"));
}

//========= UpdateVideo definitions =========

UpdateVideo::UpdateVideo() : VideoCommand("UpdateVideo")
{
}

int UpdateVideo::construct_protobuf(
    PMGDQuery& query,
    const Json::Value& jsoncmd,
    const std::string& blob,
    int grp_id,
    Json::Value& error)
{
    const Json::Value& cmd = jsoncmd[_cmd_name];

    int node_ref = get_value<int>(cmd, "_ref", -1);

    Json::Value constraints = get_value<Json::Value>(cmd, "constraints");

    Json::Value props = get_value<Json::Value>(cmd, "properties");

    Json::Value remove_props = get_value<Json::Value>(cmd, "remove_props");

    // Update Image node
    query.UpdateNode(node_ref, VDMS_VID_TAG, props,
                        remove_props,
                        constraints,
                        get_value<bool>(cmd, "unique", false));

    return 0;
}

Json::Value UpdateVideo::construct_responses(
    Json::Value& responses,
    const Json::Value& json,
    protobufs::queryMessage &query_res,
    const std::string &blob)
{
    assert(responses.size() == 1);

    Json::Value ret;

    // TODO In order to support "codec" or "operations", we could
    // implement VCL save operation here.

    ret[_cmd_name].swap(responses[0]);
    return ret;
}

//========= FindVideo definitions =========

FindVideo::FindVideo() : VideoCommand("FindVideo")
{
}

int FindVideo::construct_protobuf(
    PMGDQuery& query,
    const Json::Value& jsoncmd,
    const std::string& blob,
    int grp_id,
    Json::Value& error)
{
    const Json::Value& cmd = jsoncmd[_cmd_name];

    Json::Value results = get_value<Json::Value>(cmd, "results");

    // Unless otherwhise specified, we return the blob.
    if (get_value<bool>(results, "blob", true)){
        results["list"].append(VDMS_VID_PATH_PROP);
    }

    query.QueryNode(
            get_value<int>(cmd, "_ref", -1),
            VDMS_VID_TAG,
            cmd["link"],
            cmd["constraints"],
            results,
            get_value<bool>(cmd, "unique", false)
            );

    return 0;
}

Json::Value FindVideo::construct_responses(
    Json::Value& responses,
    const Json::Value& json,
    protobufs::queryMessage &query_res,
    const std::string &blob)
{
    const Json::Value& cmd = json[_cmd_name];

    Json::Value ret;

    auto error = [&](Json::Value& res)
    {
        ret[_cmd_name] = res;
        return ret;
    };

    Json::Value resp = check_responses(responses);
    if (resp["status"] != RSCommand::Success) {
        return error(resp);
    }

    Json::Value& FindVideo = responses[0];

    bool flag_empty = true;

    for (auto& ent : FindVideo["entities"]) {

        if(!ent.isMember(VDMS_VID_PATH_PROP)){
            continue;
        }

        std::string video_path = ent[VDMS_VID_PATH_PROP].asString();
        ent.removeMember(VDMS_VID_PATH_PROP);

        if (ent.getMemberNames().size() > 0) {
            flag_empty = false;
        }
        try {
            if (!cmd.isMember("operations") &&
                !cmd.isMember("container")  &&
                !cmd.isMember("codec"))
            {
                // Return video as is.
                std::ifstream ifile(video_path, std::ifstream::in);
                ifile.seekg(0, std::ios::end);
                size_t encoded_size = (long)ifile.tellg();
                ifile.seekg(0, std::ios::beg);

                std::string* video_str = query_res.add_blobs();
                video_str->resize(encoded_size);
                ifile.read((char*)(video_str->data()), encoded_size);
                ifile.close();
            }
            else {

                VCL::Video video(video_path);

                if (cmd.isMember("operations")) {
                    enqueue_operations(video, cmd["operations"]);
                }

                const std::string& container =
                            get_value<std::string>(cmd, "container", "mp4");
                const std::string& file_name =
                            VCL::create_unique("/tmp/", container);
                const std::string& codec =
                            get_value<std::string>(cmd, "codec", "h264");

                VCL::Video::Codec vcl_codec = string_to_codec(codec);
                video.store(file_name, vcl_codec); // to /tmp/ for encoding.

                auto video_enc = video.get_encoded();
                int size = video_enc.size();

                if (size > 0) {

                    std::string* video_str = query_res.add_blobs();
                    video_str->resize(size);
                    std::memcpy((void*)video_str->data(),
                                (void*)video_enc.data(),
                                size);
                }
                else {
                    Json::Value return_error;
                    return_error["status"]  = RSCommand::Error;
                    return_error["info"] = "Video Data not found";
                    error(return_error);
                }
            }
        } catch (VCL::Exception e) {
            print_exception(e);
            Json::Value return_error;
            return_error["status"]  = RSCommand::Error;
            return_error["info"] = "VCL Exception";
            return error(return_error);
        }
    }

    if (flag_empty) {
        FindVideo.removeMember("entities");
    }

    ret[_cmd_name].swap(FindVideo);
    return ret;
}

//========= FindFrames definitions =========

FindFrames::FindFrames() : VideoCommand("FindFrames")
{
}

bool FindFrames::get_interval_index (const Json::Value& cmd,
                                     Json::ArrayIndex& op_index)
{
    if (cmd.isMember("operations")) {
        const auto operations = cmd["operations"];
        for (auto i = 0; i < operations.size(); i++) {
            const auto op = operations[i];
            const std::string& type = get_value<std::string>(op, "type");
            if (type == "interval") {
                op_index = i;
                return true;
            }
        }
    }
    return false;
}

int FindFrames::construct_protobuf(
    PMGDQuery& query,
    const Json::Value& jsoncmd,
    const std::string& blob,
    int grp_id,
    Json::Value& error)
{
    const Json::Value& cmd = jsoncmd[_cmd_name];

    // We try to catch the missing attribute error before
    // initiating a PMGD query
    Json::ArrayIndex tmp;
    bool is_interval = get_interval_index(cmd, tmp);
    bool is_frames   = cmd.isMember("frames");

    if (!(is_frames != is_interval)) {
        error["status"]  = RSCommand::Error;
        error["info"] = "Either one of 'frames' or 'operations::interval' "
                        "must be specified";
        return -1;
    }

    Json::Value results = get_value<Json::Value>(cmd, "results");
    results["list"].append(VDMS_VID_PATH_PROP);

    query.QueryNode(
            get_value<int>(cmd, "_ref", -1),
            VDMS_VID_TAG,
            cmd["link"],
            cmd["constraints"],
            results,
            get_value<bool>(cmd, "unique", false)
            );

    // Auxiliary query to extract keyframe graph
    // random generation of ref, but it is still a chance of ref clash
    // TODO
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 32767);
    int aux_ref = dis(gen);
    results["list"].append(VDMS_VID_LABEL_PROP);

    query.QueryNode(
            aux_ref,
            VDMS_VID_TAG,
            cmd["link"],
            cmd["constraints"],
            results,
            get_value<bool>(cmd, "unique", false)
            );

    Json::Value kf_props;
    kf_props["list"].append(VDMS_VID_LABEL_PROP);
    kf_props["list"].append(VDMS_KF_IDX_PROP);
    kf_props["list"].append(VDMS_KF_PKT_POS_PROP);
    kf_props["list"].append(VDMS_KF_PKT_TS_PROP);

    Json::Value link;
    link["ref"] = aux_ref;

    Json::Value null;

    query.QueryNode(
            -1,
            VDMS_KF_TAG,
            link,
            null["null"],
            kf_props,
            false
            );
    return 0;
}

Json::Value FindFrames::construct_responses(
    Json::Value& responses,
    const Json::Value& json,
    protobufs::queryMessage &query_res,
    const std::string &blob)
{
    const Json::Value& cmd = json[_cmd_name];

    Json::Value ret;

    auto error = [&](Json::Value& res)
    {
        ret[_cmd_name] = res;
        return ret;
    };

    // 2 auxiliary query to extract keyframe
    Json::Value resp = check_responses(responses, 3);
    if (resp["status"] != RSCommand::Success) {
        return error(resp);
    }

    Json::Value& FindFrames = responses[0];

    // Aux. PMGD Resp
    Json::Value vid_resp = responses[1];
    Json::Value kf_resp  = responses[2];

    map<string, VCL::KeyFrameList> kf_map;
    map<string, string> vidpath_label_map;

    for (auto& vid_json: vid_resp["entities"])
    {
        vidpath_label_map.insert(
                    make_pair(vid_json[VDMS_VID_PATH_PROP].asString(),
                              vid_json[VDMS_VID_LABEL_PROP].asString())
                    );
    }

    for (auto& kf_json: kf_resp["entities"])
    {
        VCL::KeyFrame kf;
        kf.derivedId     = kf_json[VDMS_KF_IDX_PROP].asInt();
        kf.pkt_ts        = kf_json[VDMS_KF_PKT_TS_PROP].asInt64();
        kf.pkt_pos       = kf_json[VDMS_KF_PKT_POS_PROP].asInt64();
        string vid_label = kf_json[VDMS_VID_LABEL_PROP].asString();

        kf_map[vid_label].push_back(kf);
    }
    // Assuming keyframe list is in descending order, reverse
    for (auto& item: kf_map)
        reverse(item.second.begin(), item.second.end());

    bool flag_empty = true;

    for (auto& ent : FindFrames["entities"]) {

        std::string video_path = ent[VDMS_VID_PATH_PROP].asString();
        ent.removeMember(VDMS_VID_PATH_PROP);

        if (ent.getMemberNames().size() > 0) {
            flag_empty = false;
        }

        try {
            std::vector<unsigned int> frames;

            // Copy of operations is needed, as we pass the operations to
            // the enqueue_operations() method of ImageCommands class, and
            // it should not include 'interval' operation.
            Json::Value operations  = cmd["operations"];

            Json::ArrayIndex interval_idx;
            bool is_interval = get_interval_index(cmd, interval_idx);
            bool is_frames   = cmd.isMember("frames");

            // get indices of interested frames
            if (is_frames) {
                for (auto& fr : cmd["frames"]) {
                    frames.push_back(fr.asUInt());
                }
            }
            else if (is_interval)
            {
                Json::Value interval_op = operations[interval_idx];

                int start = get_value<int>(interval_op, "start");
                int stop  = get_value<int>(interval_op, "stop");
                int step  = get_value<int>(interval_op, "step");

                for (int i = start; i < stop; i += step)  {
                    frames.push_back(i);
                }
            }
            else {
                // This should never happen, as we check this condition in
                // FindFrames::construct_protobuf(). In case this happens, it
                // is better to signal it rather than to continue
                Json::Value return_error;
                return_error["status"]  = RSCommand::Error;
                return_error["info"] = "No 'frames' or 'interval' parameter";
                return error(return_error);
            }

            VCL::Video video(video_path);
            video.set_foi(frames);
            video.set_key_frame_list(
                        kf_map[vidpath_label_map[video_path]]);

            // By default, return frames as PNGs
            VCL::Image::Format format = VCL::Image::Format::PNG;

            FindImage img_cmd;

            if (cmd.isMember("format")) {
                format = img_cmd.get_requested_format(cmd);
            }

            if (format == VCL::Image::Format::MP4)
            {
                std::string tmp_vid = VCL::create_unique("/tmp/", "FindFrames.mp4");

                FindVideo vid_cmd;

                if (!operations.empty())
                {
                    vid_cmd.enqueue_operations(video, operations);
                }

                // We use store function to forward all the operations and
                // write out encoded stream to server disk
                video.store(tmp_vid, VCL::Video::Codec::H264);

                // Read saved tmp video into buffer and add to query blob
                std::string* vid_str = query_res.add_blobs();

                std::ifstream file;
                file.exceptions(
                    std::ifstream::badbit
                  | std::ifstream::failbit
                  | std::ifstream::eofbit);
                file.open(tmp_vid);
                file.seekg(0, std::ios::end);
                std::streampos length(file.tellg());

                if (length) {
                    file.seekg(0, std::ios::beg);
                    vid_str->resize(static_cast<std::size_t>(length));
                    file.read((char*)(vid_str->data()), static_cast<std::size_t>(length));
                }
            }
            else if ((format == VCL::Image::Format::JPG) ||
                     (format == VCL::Image::Format::PNG) ||
                     (format == VCL::Image::Format::MAT))
            {
                // for FORMAT MAT/JPG/PNG
                for (auto idx : frames) {
                    cv::Mat mat = video.get_frame(idx);
                    VCL::Image img(mat, false);

                    // We need to delete interval operations as its
                    // frame indices has been extracted to frames
                    Json::Value deleted;
                    operations.removeIndex(interval_idx, &deleted);

                    if (!operations.empty())
                    {
                        img_cmd.enqueue_operations(img, operations);
                    }

                    std::vector<unsigned char> img_enc;
                    img_enc = img.get_encoded_image(format);

                    if (!img_enc.empty())
                    {
                        std::string* img_str = query_res.add_blobs();
                        img_str->resize(img_enc.size());
                        std::memcpy((void*)img_str->data(),
                                    (void*)img_enc.data(),
                                    img_enc.size());
                    }
                    else
                    {
                        Json::Value return_error;
                        return_error["status"] = RSCommand::Error;
                        return_error["info"]   = "Image Data not found";
                        return error(return_error);
                    }
                }
            }
            else
            {
                Json::Value return_error;
                return_error["status"] = RSCommand::Error;
                return_error["info"]   = "Invalid Return Format for FindFrames";
                return error(return_error);
            }
        }
        catch (VCL::Exception e)
        {
            print_exception(e);
            Json::Value return_error;
            return_error["status"]  = RSCommand::Error;
            return_error["info"] = "VCL Exception";
            return error(return_error);
        }
    }

    if (flag_empty) {
        FindFrames.removeMember("entities");
    }

    ret[_cmd_name].swap(FindFrames);
    return ret;
}
