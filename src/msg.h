#pragma once
#include "header.h"

nlohmann::json CvtVIHeaderToJSON(vehicle_interfaces::msg::Header header);
void CvtVIHeaderToJSON(vehicle_interfaces::msg::Header header, nlohmann::json& json);

class ImageMsg;
class RecordMsg;
class BaseSubNodeMsg;


/**
 * BaseSubNodeMsg and SubNodeMsg Template Implementation
 */

class RecordMsg
{
public:
    uint8_t record_stamp_type;
    double record_stamp;
    int64_t record_stamp_offset;
    uint64_t record_frame_id;

    RecordMsg() : record_stamp_type(0), record_stamp(0), record_stamp_offset(0), record_frame_id(0) {}

    RecordMsg(const RecordMsg& msg)
    {
        this->record_stamp_type = msg.record_stamp_type;
        this->record_stamp = msg.record_stamp;
        this->record_stamp_offset = msg.record_stamp_offset;
        this->record_frame_id = msg.record_frame_id;
    }

    RecordMsg& operator=(const RecordMsg& msg)
    {
        this->record_stamp_type = msg.record_stamp_type;
        this->record_stamp = msg.record_stamp;
        this->record_stamp_offset = msg.record_stamp_offset;
        this->record_frame_id = msg.record_frame_id;
        return *this;
    }
};



/**
 * Customized Record Messages
 */

class ImageMsg
{
public:
    vehicle_interfaces::msg::Header header;
    uint8_t format_type;
    int16_t cvmat_type;
    uint8_t depth_unit_type;
    uint16_t width;
    uint16_t height;
    cv::Mat mat;

    ImageMsg& operator=(const ImageMsg& src)
    {
        this->header = src.header;
        this->format_type = src.format_type;
        this->cvmat_type = src.cvmat_type;
        this->depth_unit_type = src.depth_unit_type;
        this->width = src.width;
        this->height = src.height;
        try
        {
            this->mat = src.mat.clone();
        }
        catch (...)
        {
            this->mat = cv::Mat(100, 100, CV_8UC3, cv::Scalar(50));
        }
        return *this;
    }
};



class BaseSubNodeMsg
{
public:
    RecordMsg recordMsg;

public:
    BaseSubNodeMsg() {}

    BaseSubNodeMsg(const BaseSubNodeMsg& src)
    {
        this->recordMsg = src.recordMsg;
    }

    BaseSubNodeMsg& operator=(const BaseSubNodeMsg& src)
    {
        this->recordMsg = src.recordMsg;
        return *this;
    }

    void setBaseSubNodeMsg(const RecordMsg& msg)
    {
        this->recordMsg = msg;
    }

    virtual nlohmann::json dumpJSON()
    {
        nlohmann::json ret;
        ret["record_stamp_type"] = this->recordMsg.record_stamp_type;
        ret["record_stamp"] = this->recordMsg.record_stamp;
        ret["record_stamp_offset"] = this->recordMsg.record_stamp_offset;
        ret["record_frame_id"] = this->recordMsg.record_frame_id;
        return ret;
    }
};



template<typename T>
class SubNodeMsg : public BaseSubNodeMsg
{
public:
    T msg;

private:
    void _dumpJSON(nlohmann::json& json) {}

public:
    SubNodeMsg() {}

    SubNodeMsg(const SubNodeMsg& src) : BaseSubNodeMsg(src)
    {
        this->msg = src.msg;
    }

    SubNodeMsg& operator=(const SubNodeMsg& src)
    {
        this->msg = src.msg;
        return *this;
    }

    void setSubNodeMsg(const T& msg)
    {
        this->msg = msg;
    }

    virtual nlohmann::json dumpJSON() override
    {
        auto ret = BaseSubNodeMsg::dumpJSON();

        try
        {
            this->_dumpJSON(ret);
        }
        catch (nlohmann::json::exception& e)
        {
            std::cout << "SubNodeMsg::_dumpJSON\n";
            std::cout << e.what() << "\n";
        }
        return ret;
    }
};

template<>
void SubNodeMsg<vehicle_interfaces::msg::Distance>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["unit_type"] = this->msg.unit_type;
    json["min"] = this->msg.min;
    json["max"] = this->msg.max;
    json["distance"] = this->msg.distance;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::Environment>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["unit_type"] = this->msg.unit_type;
    json["temperature"] = this->msg.temperature;
    json["relative_humidity"] = this->msg.relative_humidity;
    json["pressure"] = this->msg.pressure;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::GPS>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["gps_status"] = this->msg.gps_status;
    json["latitude"] = this->msg.latitude;
    json["longitude"] = this->msg.longitude;
    json["altitude"] = this->msg.altitude;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::IDTable>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["idtable"] = this->msg.idtable;
}

template<>
void SubNodeMsg<ImageMsg>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["format_type"] = this->msg.format_type;
    json["cvmat_type"] = this->msg.cvmat_type;
    json["depth_unit_type"] = this->msg.depth_unit_type;
    json["width"] = this->msg.width;
    json["height"] = this->msg.height;
    // json["record_filename"] = this->msg.record_filename;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::IMU>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["unit_type"] = this->msg.unit_type;
    json["orientation"] = this->msg.orientation;
    json["angular_velocity"] = this->msg.angular_velocity;
    json["linear_acceleration"] = this->msg.linear_acceleration;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::MillitBrakeMotor>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["travel_min"] = this->msg.travel_min;
    json["travel_max"] = this->msg.travel_max;
    json["travel"] = this->msg.travel;
    json["brake_percentage"] = this->msg.brake_percentage;
    json["external_control"] = this->msg.external_control;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::MillitPowerMotor>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["motor_mode"] = this->msg.motor_mode;
    json["rpm"] = this->msg.rpm;
    json["torque"] = this->msg.torque;
    json["percentage"] = this->msg.percentage;
    json["voltage"] = this->msg.voltage;
    json["current"] = this->msg.current;
    json["temperature"] = this->msg.temperature;
    json["parking"] = this->msg.parking;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::MotorAxle>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["dir"] = this->msg.dir;
    json["pwm"] = this->msg.pwm;
    json["parking"] = this->msg.parking;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::MotorSteering>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["unit_type"] = this->msg.unit_type;
    json["min"] = this->msg.min;
    json["max"] = this->msg.max;
    json["center"] = this->msg.center;
    json["value"] = this->msg.value;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::UPS>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["volt_in"] = this->msg.volt_in;
    json["amp_in"] = this->msg.amp_in;
    json["volt_out"] = this->msg.volt_out;
    json["amp_out"] = this->msg.amp_out;
    json["temperature"] = this->msg.temperature;
}

template<>
void SubNodeMsg<vehicle_interfaces::msg::WheelState>::_dumpJSON(nlohmann::json& json)
{
    CvtVIHeaderToJSON(this->msg.header, json);
    json["gear"] = this->msg.gear;
    json["steering"] = this->msg.steering;
    json["pedal_throttle"] = this->msg.pedal_throttle;
    json["pedal_brake"] = this->msg.pedal_brake;
    json["pedal_clutch"] = this->msg.pedal_clutch;
    json["button"] = this->msg.button;
    json["func"] = this->msg.func;
}

nlohmann::json CvtVIHeaderToJSON(vehicle_interfaces::msg::Header header)
{
    nlohmann::json ret;
    ret["priority"] = header.priority;
    ret["device_type"] = header.device_type;
    ret["device_id"] = header.device_id;
    ret["frame_id"] = header.frame_id;
    ret["stamp_type"] = header.stamp_type;
    ret["stamp"] = header.stamp.sec + (double)(header.stamp.nanosec / 1000000000.0);
    ret["stamp_offset"] = header.stamp_offset;
    ret["ref_publish_time_ms"] = header.ref_publish_time_ms;
    return ret;
}

void CvtVIHeaderToJSON(vehicle_interfaces::msg::Header header, nlohmann::json& json)
{
    json["priority"] = header.priority;
    json["device_type"] = header.device_type;
    json["device_id"] = header.device_id;
    json["frame_id"] = header.frame_id;
    json["stamp_type"] = header.stamp_type;
    json["stamp"] = header.stamp.sec + (double)(header.stamp.nanosec / 1000000000.0);
    json["stamp_offset"] = header.stamp_offset;
    json["ref_publish_time_ms"] = header.ref_publish_time_ms;
}

template<typename T>
class SubSaveQueueNodeMsg : public SubNodeMsg<T>
{
public:
    std::string record_filename;

public:
    SubSaveQueueNodeMsg() : SubNodeMsg<T>() {}

    SubSaveQueueNodeMsg(const SubSaveQueueNodeMsg& src) : SubNodeMsg<T>(src)
    {
        this->record_filename = src.record_filename;
    }

    SubSaveQueueNodeMsg& operator=(const SubSaveQueueNodeMsg& src)
    {
        this->record_filename = src.record_filename;
        return *this;
    }

    void setSubNodeMsg(std::string record_filename, const T& msg)
    {
        SubNodeMsg<T>::setSubNodeMsg(msg);
        this->record_filename = record_filename;
    }

    nlohmann::json dumpJSON() override
    {
        auto ret = SubNodeMsg<T>::dumpJSON();
        ret["record_filename"] = this->record_filename;
        return ret;
    }
};