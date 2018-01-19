#include <string>
#include <fstream>
#include "QueryHandler.h"

#include "RSCommand.h"
#include "ImageCommand.h"
#include "ExceptionsCommand.h"

#include "PMGDQuery.h"
#include "jarvis.h"
#include "util.h"

#include <jsoncpp/json/writer.h>

using namespace athena;

// TODO This will be later replaced by a real logger
std::ofstream GENERIC_LOGGER("log.log", std::fstream::app);
// #define GENERIC_LOGGER std::cout

QueryHandler::QueryHandler(Jarvis::Graph *db, std::mutex *mtx)
    : _pmgd_qh(db, mtx)
{
    _rs_cmds["AddEntity"]  = new AddEntity();
    _rs_cmds["Connect"]    = new Connect();
    _rs_cmds["FindEntity"] = new FindEntity();
    _rs_cmds["AddImage"]   = new AddImage();
    _rs_cmds["FindImage"]  = new FindImage();
}

QueryHandler::~QueryHandler()
{
    for (auto cmd: _rs_cmds) {
        delete cmd.second;
    }
}

void QueryHandler::process_connection(comm::Connection *c)
{
    CommandHandler handler(c);

    try {
        while (true) {
            protobufs::queryMessage response;
            protobufs::queryMessage query = handler.get_command();
            process_query(query, response);
            handler.send_response(response);
        }
    } catch (comm::ExceptionComm e) {
        print_exception(e);
    }
}

bool QueryHandler::syntax_checker(const Json::Value &root, Json::Value& error)
{
    for (int j = 0; j < root.size(); j++) {
        const Json::Value& query = root[j];
        if (query.getMemberNames().size() != 1) {
            error["info"] = "Error: Only one command per element allowed";
            return false;
        }

        const std::string cmd_str = query.getMemberNames()[0];
        auto it = _rs_cmds.find(cmd_str);
        if (it == _rs_cmds.end()) {
            error["info"] = cmd_str + ": Command not found!";
            return false;
        }

        bool flag_error = (*it).second->check_params(query[cmd_str], error);

        if (!flag_error) {
            return false;
        }
    }

    return true;
}

int QueryHandler::parse_commands(const std::string& commands,
                                 Json::Value& root)
{
    Json::Reader reader;

    try {
        bool parseSuccess = reader.parse(commands.c_str(), root);

        if (!parseSuccess) {
            root["info"] = "Error parsing the query, ill formed JSON";
            root["status"] = RSCommand::Error;
            return -1;
        }

        Json::Value error;

        if (!syntax_checker(root, error)) {
            root = error;
            root["status"] = RSCommand::Error;
            return -1;
        }

        // Json::StyledWriter swriter;
        // GENERIC_LOGGER << swriter.write(root) << std::endl;
    } catch (Json::Exception const&) {
        root["info"] = "Json Exception at Parsing";
        root["status"] = RSCommand::Error;
        return -1;
    }

    return 0;
}

void QueryHandler::process_query(protobufs::queryMessage& proto_query,
                                 protobufs::queryMessage& proto_res)
{
    Json::FastWriter fastWriter;

    try {
        Json::Value json_responses;
        Json::Value root;

        Json::Value cmd_result;
        Json::Value cmd_current;
        bool flag_error = false;

        auto error = [&](Json::Value& res, Json::Value& failed_command)
        {
            res["FailedCommand"] = failed_command;
            json_responses.clear();
            json_responses.append(res);
            proto_res.clear_blobs();
            proto_res.set_json(fastWriter.write(json_responses));
            Json::StyledWriter w;
            GENERIC_LOGGER << w.write(json_responses);
        };

        if (parse_commands(proto_query.json(), root) != 0) {
            cmd_current = "Transaction";
            error(root, cmd_current);
            return;
        }

        PMGDQuery pmgd_query(_pmgd_qh);
        unsigned blob_count = 0;

        //iterate over the list of the queries
        for (int j = 0; j < root.size(); j++) {
            const Json::Value& query = root[j];
            assert(query.getMemberNames().size() == 1);
            std::string cmd = query.getMemberNames()[0];

            int group_count = pmgd_query.add_group();

            RSCommand *rscmd = _rs_cmds[cmd];

            const std::string& blob = rscmd->need_blob() ?
                                      proto_query.blobs(blob_count++) : "";

            int ret_code = rscmd->construct_protobuf(pmgd_query, query, blob,
                                                     group_count, cmd_result);

            if (ret_code != 0) {
                error(cmd_result, root[j]);
                return;
            }
        }

        Json::Value& tx_responses = pmgd_query.run();

        if (tx_responses.size() != root.size()) { // error
            cmd_current = "Transaction";
            cmd_result = tx_responses;
            cmd_result["info"] = "Failed PMGDTransaction";
            cmd_result["status"] = RSCommand::Error;
            error(cmd_result, cmd_current);
            return;
        }
        else {
            for (int j = 0; j < root.size(); j++) {
                std::string cmd = root[j].getMemberNames()[0];

                cmd_result = _rs_cmds[cmd]->construct_responses(
                                            tx_responses[j],
                                            root[j], proto_res);

                // This is for error handling
                if (cmd_result.isMember("status")) {
                    int status = cmd_result["status"].asInt();
                    if (status != RSCommand::Success ||
                        status != RSCommand::Empty   ||
                        status != RSCommand::Exists)
                    {
                        error(cmd_result, root[j]);
                        return;
                    }
                }
                json_responses.append(cmd_result);
            }
        }

        proto_res.set_json(fastWriter.write(json_responses));

    } catch (VCL::Exception e) {
        print_exception(e);
        GENERIC_LOGGER << "FATAL ERROR: VCL Exception at QH" << std::endl;
        exit(0);
    } catch (Jarvis::Exception e) {
        print_exception(e);
        GENERIC_LOGGER << "FATAL ERROR: PMGD Exception at QH" << std::endl;
        exit(0);
    } catch (ExceptionCommand e) {
        print_exception(e);
        GENERIC_LOGGER << "FATAL ERROR: Command Exception at QH" << std::endl;
        exit(0);
    } catch (Json::Exception const&) {
        // Should not happen
        // In case of error on the last fastWriter
        GENERIC_LOGGER << "FATAL: Json Exception!" << std::endl;
        Json::Value error;
        error["info"] = "Internal Server Error: Json Exception";
        error["status"] = RSCommand::Error;
        proto_res.set_json(fastWriter.write(error));
    } catch (const std::invalid_argument& ex) {
        GENERIC_LOGGER << "Invalid argument: " << ex.what() << '\n';
        exit(0);
    }
}
