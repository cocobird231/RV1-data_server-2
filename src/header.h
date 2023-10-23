#pragma once
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

// Local header
#include "msg.h"

//#define NODE_SUBSCRIBE_PRINT

using namespace std::chrono_literals;
using namespace std::placeholders;

enum ROS2InterfaceType { MSG, SRV };

class ROS2InterfaceInfo
{
public:
    std::string packName;
    ROS2InterfaceType type;
    std::string msgType;

    ROS2InterfaceInfo& operator=(const ROS2InterfaceInfo& src)
    {
        this->packName = src.packName;
        this->type = src.type;
        this->msgType = src.msgType;
        return *this;
    }
};

class TopicInfo
{
public:
    std::string topicName;
    ROS2InterfaceInfo interface; 
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
 * BaseSubNode Implementation and SubNode Template Specialization
 */

class BaseSubNode : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    bool initF_;
    bool storeF_;

public:
    BaseSubNode(std::string nodeName, std::string qosServiceName, std::string qosDirPath) : 
        vehicle_interfaces::PseudoTimeSyncNode(nodeName), 
        vehicle_interfaces::QoSUpdateNode(nodeName, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName), 
        initF_(false), 
        storeF_(true)
    {

    }

    void setInitF(bool flag) { this->initF_ = flag; }
    
    bool isInit() const { return this->initF_; }

    void setStoreFlag(bool flag) { this->storeF_ = flag; }

    bool getStoreFlag() const { return this->storeF_; }

    virtual bool dataAvailable() { return false; }

    virtual bool getData(std::deque<std::shared_ptr<BaseSubNodeMsg> >& msgQue, bool& newData, bool force) { return false; }
};

template<typename T, typename U>
class SubNode : public BaseSubNode
{
private:
    std::string nodeName_;
    TopicInfo topicInfo_;
    std::shared_ptr<rclcpp::Subscription<T> > subscription_;
    std::mutex nodeLock_;

    std::atomic<uint64_t> subFrameID_;

    SubNodeMsg<U> data_;
    SubNodeMsg<U> dataBk_;
    SubNodeMsg<U>* dataFilledPtr_;
    SubNodeMsg<U>* dataGetPtr_;
    std::atomic<bool> newDataF_;
    std::mutex dataPtrLock_;
    std::atomic<bool> dataAvailableF_;

private:
    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        std::unique_lock<std::mutex> nodeLocker(this->nodeLock_, std::defer_lock);

        nodeLocker.lock();
        auto topicName = this->topicInfo_.topicName;
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
        if (!this->getStoreFlag())
            return;

        RecordMsg recordMsg;
        recordMsg.record_stamp_type = this->getTimestampType();
        recordMsg.record_stamp = this->getTimestamp().seconds();
        recordMsg.record_stamp_offset = static_cast<int64_t>(this->getCorrectDuration().nanoseconds());
        recordMsg.record_frame_id = this->subFrameID_++;
        this->dataFilledPtr_->setRecordMsg(recordMsg);

        this->_msgProc(msg, this->dataFilledPtr_);

        std::lock_guard<std::mutex> locker(this->dataPtrLock_);
        std::swap(this->dataFilledPtr_, this->dataGetPtr_);
        this->newDataF_ = true;

        if (!this->dataAvailableF_)
            this->dataAvailableF_ = true;
    }

    virtual void _msgProc(const std::shared_ptr<T> msg, SubNodeMsg<U>* ptr)
    {
        RCLCPP_WARN(this->get_logger(), "[SubNode::_msgProc (%s)] Function not override.", this->nodeName_.c_str());
    }

public:
    SubNode<T, U>(std::string nodeName, TopicInfo topicInfo, std::string qosServiceName, std::string qosDirPath) : 
        BaseSubNode(nodeName, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName)
    {
        this->nodeName_ = nodeName;
        this->topicInfo_ = topicInfo;
        this->dataFilledPtr_ = &this->data_;
        this->dataGetPtr_ = &this->dataBk_;

        this->subFrameID_ = 0;

        this->addQoSCallbackFunc(std::bind(&SubNode::_qosCallback, this, std::placeholders::_1));
        vehicle_interfaces::QoSPair qpair = this->addQoSTracking(this->topicInfo_.topicName);
        this->subscription_ = this->create_subscription<T>(this->topicInfo_.topicName, 
            *qpair.second, std::bind(&SubNode::_topicCallback, this, std::placeholders::_1));
    }

    bool dataAvailable() { return this->dataAvailableF_; }
    
    // Return false if msg data not available. msgQue will not be pushed if msg data not available.
    // msgQue pushed if newData arrived or force flag set to true.
    virtual bool getData(std::deque<std::shared_ptr<BaseSubNodeMsg> >& msgQue, bool& newData, bool force) override
    {
        std::lock_guard<std::mutex> locker(this->dataPtrLock_);

        bool ret = this->newDataF_;
        newData = this->newDataF_;

        this->newDataF_ = false;
        this->subFrameID_ = 0;

        if (!this->dataAvailableF_)
            return false;
        if (!ret && !force)
            return true;

        msgQue.push_back(std::make_shared<SubNodeMsg<U> >(*this->dataGetPtr_));

        if (ret && this->topicInfo_.interface.msgType == "Image")// Tmp solution for Image msg
        {
            if (SubNodeMsg<ImageMsg>* imgMsg = dynamic_cast<SubNodeMsg<ImageMsg>*>(this->dataGetPtr_))
            {
                imgMsg->msg.mat = cv::Mat::zeros(imgMsg->msg.mat.size(), imgMsg->msg.mat.type());
            }
        }
        return true;
    }
};

template<typename T, typename U, typename V>
class SubSaveQueueNode : public SubNode<T, U>
{
private:
    SaveQueue<T>* saveImgQue_;

public:
    SubSaveQueueNode<T, U, V>(std::string nodeName, TopicInfo topicInfo, SaveQueue<V>* saveQue, std::string qosServiceName, std::string qosDirPath) : 
        SubNode<T, U>(nodeName, topicInfo, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName)
    {
        this->saveImgQue_ = saveQue;
    }

    bool getData(std::deque<std::shared_ptr<BaseSubNodeMsg> >& msgQue, bool& newData, bool force) override
    {
        std::lock_guard<std::mutex> locker(this->dataPtrLock_);

        bool ret = this->newDataF_;
        newData = this->newDataF_;

        this->newDataF_ = false;
        this->subFrameID_ = 0;

        if (!this->dataAvailableF_)
            return false;
        if (!ret && !force)
            return true;

        msgQue.push_back(std::make_shared<SubNodeMsg<U> >(*this->dataGetPtr_));
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

    std::string record_filename;
    auto ts = static_cast<rclcpp::Time>(msg->header.stamp).seconds();
    if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
        record_filename = nodeName + "/" + std::to_string(ts) + ".jpg";
    else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW)
        record_filename = nodeName + "/" + std::to_string(ts) + ".tiff";

    ImageMsg imgMsg;
    imgMsg.header = msg->header;
    imgMsg.format_type = msg->format_type;
    imgMsg.cvmat_type = msg->cvmat_type;
    imgMsg.depth_unit_type = msg->depth_unit_type;
    imgMsg.width = msg->width;
    imgMsg.height = msg->height;
    imgMsg.record_filename = record_filename;
    imgMsg.mat = recvMat.clone();
    ptr->msg = imgMsg;
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