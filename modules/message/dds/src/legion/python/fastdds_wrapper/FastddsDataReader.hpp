// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file FastddsDataReader.hpp
 *
 */

#ifndef _FASTDDSDATAREADER_HPP_
#define _FASTDDSDATAREADER_HPP_

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <string>
#include <stdexcept>
#include "fastdds/rtps/transport/UDPv4TransportDescriptor.h"
#include <fastrtps/rtps/attributes/HistoryAttributes.h>
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

template<class CustomTopicDataType>
class FastddsDataReader
{

public:

    typedef typename CustomTopicDataType::type type;

    FastddsDataReader(
            uint32_t did,
            const std::string& topic_name)
        : participant_(nullptr)
        , topic_(nullptr)
        , subscriber_(nullptr)
        , datareader_(nullptr)
        , topic_name_(topic_name)
        , type_(new CustomTopicDataType())
        , did_(did)
    {
        init();
    }

    ~FastddsDataReader()
    {
        destroy();
    }

    bool take_sample(
            type& msg)
    {
        eprosima::fastdds::dds::SampleInfo info;
        if (datareader_->take_next_sample(&msg, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE)
            {
                return true;
            }
        }

        return false;
    }

    bool wait_for_sample(uint32_t seconds)
    {
        eprosima::fastrtps::Duration_t timeout(seconds, 0);
        return datareader_->wait_for_unread_message(timeout);
    }


private:


    void init()
    {
        DomainParticipantQos pqos;
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        udp_transport->sendBufferSize = 0;
        udp_transport->receiveBufferSize = 0;
        udp_transport->TTL = 3;
        udp_transport->non_blocking_send = true;
        // Link the Transport Layer to the Participant.
        pqos.transport().user_transports.push_back(udp_transport);
        // Avoid using the default transport
        pqos.transport().use_builtin_transports = false;
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
            DiscoveryProtocol_t::SIMPLE;
        pqos.wire_protocol()
            .builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
        pqos.wire_protocol()
            .builtin.discovery_config.m_simpleEDP
            .use_PublicationReaderANDSubscriptionWriter = true;
        pqos.wire_protocol()
            .builtin.discovery_config.m_simpleEDP
            .use_PublicationWriterANDSubscriptionReader = true;
        pqos.name("Participant_pub");

        participant_ = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(
                did_,pqos);

        if (participant_ == nullptr)
        {
            throw new std::runtime_error("Error creating participant");
        }

        // Register type
        type_.register_type(participant_);

        // Create topic
        topic_ = participant_->create_topic(topic_name_, type_->getName(),
                        eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
    

        if (topic_ == nullptr)
        {
            std::stringstream error;
            error << "Error creating topic [" << topic_name_ << "]";
            throw new std::runtime_error(error.str());
        }

        // Create subscriber
        subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
        if (subscriber_ == nullptr)
        {
            throw new std::runtime_error("Error creating subscriber");
        }

        DataReaderQos rqos;
        rqos.history().kind = KEEP_LAST_HISTORY_QOS;
        rqos.history().depth = 10;
        rqos.resource_limits().max_samples = 10000;
        //rqos.resource_limits().allocated_samples = 400;
        //rqos.resource_limits().max_instances = 100;
        rqos.reliable_reader_qos().times.heartbeatResponseDelay.seconds = 1;
        rqos.reliable_reader_qos().times.heartbeatResponseDelay.fraction(0);
        // rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;
        rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        rqos.endpoint().history_memory_policy = DYNAMIC_REUSABLE_MEMORY_MODE;

        // Create datareader
        datareader_ = subscriber_->create_datareader(topic_, rqos);
        if (datareader_ == nullptr)
        {
            throw new std::runtime_error("Error creating datareader");
        }
    }

    void destroy()
    {
        if (participant_)
        {
            if (datareader_)
            {
                subscriber_->delete_datareader(datareader_);
                datareader_ = nullptr;
            }
            if (subscriber_)
            {
                participant_->delete_subscriber(subscriber_);
                subscriber_ = nullptr;
            }
            if (topic_)
            {
                participant_->delete_topic(topic_);
                topic_ = nullptr;
            }
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

    FastddsDataReader(
            const FastddsDataReader&) = delete;
    FastddsDataReader& operator =(
            const FastddsDataReader&);

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::DataReader* datareader_;
    std::string topic_name_;
    eprosima::fastdds::dds::TypeSupport type_;
    uint32_t did_;
};

#endif // _FASTDDSDATAREADER_HPP_
