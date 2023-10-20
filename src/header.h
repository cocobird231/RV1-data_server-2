#include <chrono>
#include <functional>
#include <memory>

#include <map>
#include <array>
#include <deque>
#include <vector>
#include <string>

#include <thread>
#include <mutex>
#include <atomic>

#include <sstream>
#include <algorithm>
#include <regex>

// JSON File
#include <sqlite3.h>
#include <nlohmann/json.hpp>
#include <iomanip>// nlohmann json dependency
#include <fstream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg/distance.hpp"
#include "vehicle_interfaces/msg/environment.hpp"
#include "vehicle_interfaces/msg/gps.hpp"
#include "vehicle_interfaces/msg/ground_detect.hpp"
#include "vehicle_interfaces/msg/id_table.hpp"
#include "vehicle_interfaces/msg/image.hpp"
#include "vehicle_interfaces/msg/imu.hpp"
#include "vehicle_interfaces/msg/millit_brake_motor.hpp"
#include "vehicle_interfaces/msg/millit_power_motor.hpp"
#include "vehicle_interfaces/msg/motor_axle.hpp"
#include "vehicle_interfaces/msg/motor_steering.hpp"
#include "vehicle_interfaces/msg/ups.hpp"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#include "vehicle_interfaces/srv/id_server.hpp"
#include "vehicle_interfaces/vehicle_interfaces.h"

// Image Process
#include <opencv2/opencv.hpp>

//#define NODE_SUBSCRIBE_PRINT

using namespace std::chrono_literals;
using namespace std::placeholders;

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

    std::string fileName;
    cv::Mat mat = cv::Mat(100, 100, CV_8UC3, cv::Scalar(50));

    ImageMsg& operator=(const ImageMsg& src)
    {
        this->header = src.header;
        this->format_type = src.format_type;
        this->cvmat_type = src.cvmat_type;
        this->depth_unit_type = src.depth_unit_type;
        this->width = src.width;
        this->height = src.height;
        this->fileName = src.fileName;
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

template<typename T>
using PairQue = std::deque<std::pair<std::string, T>>;

template<typename T>
class SaveQueue
{
private:
    std::deque<PairQue<T>> totalPairQue_;

    std::vector<std::thread> thVec_;
    std::vector<std::timed_mutex> queLockVec_;
    size_t thNum_;

    std::atomic<size_t> thSelect_;
    std::mutex thSelectLock_;

    int interval_ms_;

    std::atomic<bool> exitF_;

private:
    void _saveTh(size_t queID)// specified
    {
        std::unique_lock<std::timed_mutex> locker(this->queLockVec_[queID], std::defer_lock);
        PairQue<T>* PairQuePtr = &this->totalPairQue_[queID];
        bool emptyF = true;
        while (!(this->exitF_ && emptyF))
        {
            if (locker.try_lock_for(5ms))
            {
                if (PairQuePtr->empty())
                {
                    emptyF = true;
                    locker.unlock();
                    std::this_thread::sleep_for(std::chrono::milliseconds(this->interval_ms_));
                    continue;
                }
                PairQuePtr->pop_front();
                emptyF = false;
                locker.unlock();
                std::cerr << "!!!SaveQueue Unspecified!!!" << "\n";
            }
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(this->interval_ms_));
        }
    }

    size_t _getSaveQueID()
    {
        std::unique_lock<std::mutex> locker(this->thSelectLock_, std::defer_lock);
        size_t ret;
        locker.lock();
        ret = (++this->thSelect_) % this->thNum_;
        this->thSelect_ = ret;
        locker.unlock();
        return ret;
    }

public:
    SaveQueue(size_t thNum = 1, int scanInterval_ms = 100) : exitF_(false), thSelect_(0)
    {
        this->thNum_ = thNum;
        this->interval_ms_ = scanInterval_ms;
        
        this->queLockVec_ = std::vector<std::timed_mutex>(thNum);
        this->totalPairQue_ = std::deque<PairQue<T>>(thNum);

        for (size_t i = 0; i < thNum; i++)
            this->thVec_.emplace_back(&SaveQueue::_saveTh, this, i);
    }

    ~SaveQueue()
    {
        this->exitF_ = true;
        for (size_t i = 0; i < this->thNum_; i++)
            this->thVec_[i].join();
    }

    void push(const std::string& fileName, const T& element)
    {
        auto id = _getSaveQueID();
        std::unique_lock<std::timed_mutex> locker(this->queLockVec_[id], std::defer_lock);
        locker.lock();
        this->totalPairQue_[id].emplace_back(fileName, element);
        locker.unlock();
    }

    std::vector<size_t> getSize()
    {
        std::vector<size_t> ret(thNum_, 0);
        for (size_t i = 0; i < this->thNum_; i++)
        {
            std::unique_lock<std::timed_mutex> locker(this->queLockVec_[i], std::defer_lock);
            locker.lock();
            ret[i] = this->totalPairQue_[i].size();
            locker.unlock();
        }
        return ret;
    }

    void shrink_to_fit()
    {
        for (size_t i = 0; i < this->thNum_; i++)
        {
            std::unique_lock<std::timed_mutex> locker(this->queLockVec_[i], std::defer_lock);
            locker.lock();
            //this->totalPairQue_[i].shrink_to_fit();
            PairQue<T>(this->totalPairQue_[i]).swap(this->totalPairQue_[i]);
            locker.unlock();
        }
    }

    void close() { this->exitF_ = true; }
};

template<>
void SaveQueue<cv::Mat>::_saveTh(size_t queID)// cv::Mat specified TODO: to be validation
{
    std::unique_lock<std::timed_mutex> locker(this->queLockVec_[queID], std::defer_lock);
    PairQue<cv::Mat>* PairQuePtr = &this->totalPairQue_[queID];
    bool emptyF = true;
    std::string fileName;
    cv::Mat element;
    while (!(this->exitF_ && emptyF))
    {
        if (locker.try_lock_for(5ms))
        {
            if (PairQuePtr->empty())
            {
                emptyF = true;
                locker.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(this->interval_ms_));
                continue;
            }
            const std::pair<std::string, cv::Mat>& item = PairQuePtr->front();
            fileName = item.first;
            element = item.second.clone();
            PairQuePtr->pop_front();
            emptyF = false;
            locker.unlock();
            cv::imwrite(fileName, element);
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(this->interval_ms_));
    }
}


/**
 * BaseSubNodeMsg and SubNodeMsg Template Implementation
 */

class BaseSubNodeMsg
{
public:
    uint8_t record_stamp_type;
    double record_stamp;
    int64_t record_stamp_offset;
    uint64_t record_frame_id;

    virtual nlohmann::json dumpJSON()
    {
        nlohmann::json ret;
        ret["record_stamp_type"] = record_stamp_type;
        ret["record_stamp"] = record_stamp;
        ret["record_stamp_offset"] = record_stamp_offset;
        ret["record_frame_id"] = record_frame_id;
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
    nlohmann::json dumpJSON() override
    {
        auto ret = BaseSubNodeMsg::dumpJSON();
        this->_dumpJSON(ret);
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
    json["fileName"] = this->msg.fileName;
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


/**
 * BaseSubNode Implementation and SubNode Template Specialization
 */

class BaseSubNode : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    bool initF_;
    bool showF_;

public:
    BaseSubNode(std::string nodeName, std::string qosServiceName, std::string qosDirPath) : 
        vehicle_interfaces::PseudoTimeSyncNode(nodeName), 
        vehicle_interfaces::QoSUpdateNode(nodeName, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName), 
        initF_(false), 
        showF_(true)
    {

    }

    void setInitF(bool flag) { this->initF_ = flag; }
    
    bool isInit() const { return this->initF_; }

    void show(bool flag) { this->showF_ = flag; }

    bool isShow() const { return this->showF_; }

    virtual bool getData(BaseSubNodeMsg* msg) { return false; }

    virtual bool getData(std::deque<std::shared_ptr<BaseSubNodeMsg> >& msgQue) { return false; }
};

template<typename T, typename U>
class SubNode : public BaseSubNode
{
private:
    std::string nodeName_;
    std::string topicName_;
    std::shared_ptr<rclcpp::Subscription<T> > subscription_;
    std::mutex nodeLock_;

    std::atomic<uint64_t> subFrameID_;

    SubNodeMsg<U> data_;
    SubNodeMsg<U> dataBk_;
    SubNodeMsg<U>* dataFilledPtr_;
    SubNodeMsg<U>* dataGetPtr_;
    std::atomic<bool> newDataF_;
    std::mutex dataPtrLock_;

private:
    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        std::unique_lock<std::mutex> nodeLocker(this->nodeLock_, std::defer_lock);

        nodeLocker.lock();
        auto topicName = this->topicName_;
        nodeLocker.unlock();

        for (const auto& [k, v] : qmap)
        {
            if (k == topicName || k == (std::string)this->get_namespace() + "/" + topicName)
            {
                nodeLocker.lock();
                this->subscription_.reset();
                this->subscription_ = this->create_subscription<T>(topicName, 
                    *v, std::bind(&SubNode::_topicCallback, this, std::placeholders::_1));
                nodeLocker.unlock();
            }
        }
    }

    void _topicCallback(const std::shared_ptr<T> msg)
    {
        if (!this->isShow())
            return;

        this->dataFilledPtr_->record_stamp_type = this->getTimestampType();
        this->dataFilledPtr_->record_stamp = this->getTimestamp().seconds();
        this->dataFilledPtr_->record_stamp_offset = static_cast<int64_t>(this->getCorrectDuration().nanoseconds());
        this->dataFilledPtr_->record_frame_id = this->subFrameID_++;

        this->_msgProc(msg, this->dataFilledPtr_);

        std::lock_guard<std::mutex> locker(this->dataPtrLock_);
        std::swap(this->dataFilledPtr_, this->dataGetPtr_);
        this->newDataF_ = true;
    }

    virtual void _msgProc(const std::shared_ptr<T> msg, SubNodeMsg<U>* ptr)
    {
        RCLCPP_WARN(this->get_logger(), "[SubNode::_msgProc (%s)] Function not override.", this->nodeName_.c_str());
    }

public:
    SubNode<T, U>(std::string nodeName, std::string topicName,std::string qosServiceName, std::string qosDirPath) : 
        BaseSubNode(nodeName, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName)
    {
        this->nodeName_ = nodeName;
        this->topicName_ = topicName;
        this->dataFilledPtr_ = &this->data_;
        this->dataGetPtr_ = &this->dataBk_;

        this->subFrameID_ = 0;

        this->addQoSCallbackFunc(std::bind(&SubNode::_qosCallback, this, std::placeholders::_1));
        vehicle_interfaces::QoSPair qpair = this->addQoSTracking(topicName);
        this->subscription_ = this->create_subscription<T>(topicName, 
            *qpair.second, std::bind(&SubNode::_topicCallback, this, std::placeholders::_1));
    }
    
    // Get latest subscribed topic data. Function return true if new data arrived and update argument, otherwise return false and do nothing.
    bool getData(SubNodeMsg<U>& data)
    {
        if (!this->newDataF_)
            return false;
        this->newDataF_ = false;

        std::lock_guard<std::mutex> locker(this->dataPtrLock_);
        data = *this->dataGetPtr_;
        this->subFrameID_ = 0;
        return true;
    }

    bool getData(BaseSubNodeMsg* data)
    {
        if (!this->newDataF_)
            return false;
        this->newDataF_ = false;

        std::lock_guard<std::mutex> locker(this->dataPtrLock_);
        data = new SubNodeMsg<U>(*this->dataGetPtr_);
        this->subFrameID_ = 0;
        return true;
    }

    bool getData(std::deque<std::shared_ptr<BaseSubNodeMsg> >& msgQue)
    {
        if (!this->newDataF_)
            return false;
        this->newDataF_ = false;

        std::unique_lock<std::mutex> locker(this->dataPtrLock_, std::defer_lock);
        locker.lock();
        // msgQue.push_back(new SubNodeMsg<U>(*this->dataGetPtr_));
        msgQue.push_back(std::make_shared<SubNodeMsg<U> >(*this->dataGetPtr_));
        locker.unlock();
        this->subFrameID_ = 0;
        return true;
    }

};

template<>
void SubNode<vehicle_interfaces::msg::Distance, vehicle_interfaces::msg::Distance>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::Distance> msg, SubNodeMsg<vehicle_interfaces::msg::Distance>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::Environment, vehicle_interfaces::msg::Environment>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::Environment> msg, SubNodeMsg<vehicle_interfaces::msg::Environment>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::GPS, vehicle_interfaces::msg::GPS>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::GPS> msg, SubNodeMsg<vehicle_interfaces::msg::GPS>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::IDTable, vehicle_interfaces::msg::IDTable>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::IDTable> msg, SubNodeMsg<vehicle_interfaces::msg::IDTable>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::Image, ImageMsg>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::Image> msg, SubNodeMsg<ImageMsg>* ptr)
{
    cv::Mat recvMat(100, 100, CV_8UC3, cv::Scalar(50));
    if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
    {
        recvMat = cv::imdecode(msg->data, 1);
        cv::cvtColor(recvMat, recvMat, cv::COLOR_BGRA2BGR);
    }
    else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW && msg->cvmat_type == CV_32FC1)
    {
        float *depths = reinterpret_cast<float *>(&msg->data[0]);
        recvMat = cv::Mat(msg->height, msg->width, CV_32FC1, depths);
    }
    else
        return;

    std::unique_lock<std::mutex> nodeLocker(this->nodeLock_, std::defer_lock);
    nodeLocker.lock();
    auto nodeName = this->nodeName_;
    nodeLocker.unlock();

    std::string fileName;
    auto ts = static_cast<rclcpp::Time>(msg->header.stamp).seconds();
    if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
        fileName = nodeName + "/" + std::to_string(ts) + ".jpg";
    else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW)
        fileName = nodeName + "/" + std::to_string(ts) + ".tiff";

    ptr->msg.header = msg->header;
    ptr->msg.format_type = msg->format_type;
    ptr->msg.cvmat_type = msg->cvmat_type;
    ptr->msg.depth_unit_type = msg->depth_unit_type;
    ptr->msg.width = msg->width;
    ptr->msg.height = msg->height;
    ptr->msg.fileName = fileName;
    ptr->msg.mat = recvMat.clone();
}

template<>
void SubNode<vehicle_interfaces::msg::IMU, vehicle_interfaces::msg::IMU>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::IMU> msg, SubNodeMsg<vehicle_interfaces::msg::IMU>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::MillitBrakeMotor, vehicle_interfaces::msg::MillitBrakeMotor>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::MillitBrakeMotor> msg, SubNodeMsg<vehicle_interfaces::msg::MillitBrakeMotor>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::MillitPowerMotor, vehicle_interfaces::msg::MillitPowerMotor>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::MillitPowerMotor> msg, SubNodeMsg<vehicle_interfaces::msg::MillitPowerMotor>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::MotorAxle, vehicle_interfaces::msg::MotorAxle>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::MotorAxle> msg, SubNodeMsg<vehicle_interfaces::msg::MotorAxle>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::MotorSteering, vehicle_interfaces::msg::MotorSteering>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::MotorSteering> msg, SubNodeMsg<vehicle_interfaces::msg::MotorSteering>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::UPS, vehicle_interfaces::msg::UPS>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::UPS> msg, SubNodeMsg<vehicle_interfaces::msg::UPS>* ptr)
{
    ptr->msg = *msg;
}

template<>
void SubNode<vehicle_interfaces::msg::WheelState, vehicle_interfaces::msg::WheelState>::_msgProc(const std::shared_ptr<vehicle_interfaces::msg::WheelState> msg, SubNodeMsg<vehicle_interfaces::msg::WheelState>* ptr)
{
    ptr->msg = *msg;
}


/**
 * Functions
 */

void SpinSubNodeExecutor(rclcpp::executors::SingleThreadedExecutor* exec, std::shared_ptr<BaseSubNode> node, std::string threadName)
{
    std::this_thread::sleep_for(1s);
    node->setInitF(true);
    std::cerr << threadName << " start..." << std::endl;
    exec->spin();
    std::cerr << threadName << " exit." << std::endl;
}

void SpinExecutor(rclcpp::executors::SingleThreadedExecutor* exec, std::string threadName)
{
    std::this_thread::sleep_for(1s);
    std::cerr << threadName << " start..." << std::endl;
    exec->spin();
    std::cerr << threadName << " exit." << std::endl;
}