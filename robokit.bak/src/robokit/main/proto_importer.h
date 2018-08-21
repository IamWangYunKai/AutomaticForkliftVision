#ifndef PROTOIMPORTER_H_
#define PROTOIMPORTER_H_

#include <robokit/core/singleton.h>

#include <string>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/compiler/importer.h>

class ProtoImporter
{
public:
    ProtoImporter();
public:
    bool Import(const std::string& filename);
    google::protobuf::Message* createDynamicMessage(const std::string& typeName);
    const google::protobuf::DescriptorPool* getPool() { return importer.pool(); }
public:
    google::protobuf::compiler::Importer importer;
    google::protobuf::DynamicMessageFactory factory;
};

#define sProtoImporter rbk::core::NormalSingleton<ProtoImporter>::Instance()

#endif
