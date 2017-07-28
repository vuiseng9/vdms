#pragma once
#include <string>
#include <mutex>
#include <vector>
#include <unordered_map>
#include "VCL.h"

#include "protobuf/queryMessage.pb.h" // Protobuf implementation
#include "CommandHandler.h"
#include "PMGDQueryHandler.h" // to provide the database connection

// Json parsing files
#include <jsoncpp/json/value.h>

namespace athena {
    // Instance created per worker thread to handle all transactions on a given
    // connection.

    class RSCommand {

    public:

        virtual int construct_protobuf(
                            std::vector<pmgd::protobufs::Command*> &cmds,
                            const Json::Value& root,
                            const std::string& blob,
                            int txid) = 0;

        // virtual Json::Value send_response();

        void run_operations(VCL::Image& vclimg, const Json::Value& op);

        virtual bool need_blob(){return false;}
    protected:
         void check_properties_type(pmgd::protobufs::Property *p,
                                    const char * prop_name,
                                    Json::Value );
     };

    // Low-level API
    class AddNode: public RSCommand {

    public:
        int construct_protobuf( std::vector<pmgd::protobufs::Command*> &cmds,
                                const Json::Value& root,
                                const std::string& blob,
                                int txid);

        bool need_blob(){return false;}
        // Json::Value send_response();
    };

    class AddEdge: public RSCommand {

    public:
        int construct_protobuf( std::vector<pmgd::protobufs::Command*> &cmds,
                                const Json::Value& root,
                                const std::string& blob,
                                int txid);
        bool need_blob(){return false;}
        // Json::Value send_response();
    };

    // High-level API
    class AddImage: public RSCommand {

    public:
        int construct_protobuf( std::vector<pmgd::protobufs::Command*> &cmds,
                                const Json::Value& root,
                                const std::string& blob,
                                int txid);
        bool need_blob(){return true;}
        // Json::Value send_response();
    };

    class QueryHandler
    {
        PMGDQueryHandler _pmgd_qh;
        std::unordered_map<std::string, RSCommand *> _rs_cmds;


    public:
        QueryHandler(Jarvis::Graph *db, std::mutex *mtx);
        void process_connection(comm::Connection *c);
        void process_query(protobufs::queryMessage proto_query,
                           protobufs::queryMessage& response);
    };
};
