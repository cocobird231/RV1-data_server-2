#include "header.h"

// #define LESS_COPY_METHOD

/**
 * Params Class Implementation.
 */
class Params : public vehicle_interfaces::GenericParams
{
private:
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _paramsCallbackHandler;
    std::function<void(const rclcpp::Parameter)> cbFunc_;
    std::atomic<bool> cbFuncSetF_;

public:
    std::vector<std::string> msg_filter = { "vehicle_interfaces" };
    double scan_period_ms = 1000.0;
    double sampling_period_ms = 10.0;
    double dump_period_s = 60.0;
    double record_duration_s = -1.0;
    int img_threads = 4;
    int gnd_threads = 1;
    std::string dump_path;
    bool enable_control = true;

    bool control_enable_monitor = false;
    bool control_enable_record = false;
    double control_scan_period_ms = 1000.0;
    double control_sampling_period_ms = 10.0;
    double control_dump_period_s = 60.0;
    double control_record_duration_s = -1.0;

private:
    void _getParams()
    {
        this->get_parameter("msg_filter", this->msg_filter);
        this->get_parameter("scan_period_ms", this->scan_period_ms);
        this->get_parameter("sampling_period_ms", this->sampling_period_ms);
        this->get_parameter("dump_period_s", this->dump_period_s);
        this->get_parameter("record_duration_s", this->record_duration_s);
        this->get_parameter("img_threads", this->img_threads);
        this->get_parameter("gnd_threads", this->gnd_threads);
        this->get_parameter("dump_path", this->dump_path);
        this->get_parameter("enable_control", this->enable_control);

        this->get_parameter("control_enable_monitor", this->control_enable_monitor);
        this->get_parameter("control_enable_record", this->control_enable_record);
        this->get_parameter("control_scan_period_ms", this->control_scan_period_ms);
        this->get_parameter("control_sampling_period_ms", this->control_sampling_period_ms);
        this->get_parameter("control_dump_period_s", this->control_dump_period_s);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult ret;
        ret.successful = true;
        ret.reason = "";

        if (!this->cbFuncSetF_)
            return ret;

        for (const auto& param : params)
        {
            try
            {
                this->cbFunc_(param);
            }
            catch (...)
            {
                ret.successful = false;
                ret.reason = "[Params::_paramsCallback] Caught Unknown Exception!";
            }
        }

        return ret;
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName), 
        cbFuncSetF_(false)
    {
        this->declare_parameter<std::vector<std::string> >("msg_filter", this->msg_filter);
        this->declare_parameter<double>("scan_period_ms", this->scan_period_ms);
        this->declare_parameter<double>("sampling_period_ms", this->sampling_period_ms);
        this->declare_parameter<double>("dump_period_s", this->dump_period_s);
        this->declare_parameter<double>("record_duration_s", this->record_duration_s);
        this->declare_parameter<int>("img_threads", this->img_threads);
        this->declare_parameter<int>("gnd_threads", this->gnd_threads);
        this->declare_parameter<std::string>("dump_path", this->dump_path);
        this->declare_parameter<bool>("enable_control", this->enable_control);

        this->declare_parameter<bool>("control_enable_monitor", this->control_enable_monitor);
        this->declare_parameter<bool>("control_enable_record", this->control_enable_record);
        this->declare_parameter<double>("control_scan_period_ms", this->control_scan_period_ms);
        this->declare_parameter<double>("control_sampling_period_ms", this->control_sampling_period_ms);
        this->declare_parameter<double>("control_dump_period_s", this->control_dump_period_s);
        this->_getParams();

        this->_paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Params::_paramsCallback, this, std::placeholders::_1));
    }

    void addCallbackFunc(const std::function<void(const rclcpp::Parameter)>& callback)
    {
        this->cbFunc_ = callback;
        this->cbFuncSetF_ = true;
    }
};

struct TopicContainer
{
    TopicInfo info;
    bool occupyF;
    std::shared_ptr<BaseSubNode> node;
    std::pair<std::shared_ptr<BaseSubNodeMsg>, int64_t> msgBuf = { nullptr, -1 };// Store latest msg
};

typedef std::map<std::string, TopicContainer> TopicContainerPack;// Format: {"topicName" : TopicContainer, ...}.

struct Recorder
{
    std::deque<std::shared_ptr<BaseSubNodeMsg> > msgQue;// Record sampling data for all topics.
    std::map<std::string, std::map<std::string, int64_t> > recordMap;// Sampling timestamp and index pair corresponds to msgQue. Format: {"sampTs" : {"topicName" : idx, ...}, ...}.
};

class RecordNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    // Parameters
    std::shared_ptr<Params> params_;
    std::set<std::string> filterSet_;

    // Subscriber threads
    std::map<std::string, rclcpp::executors::SingleThreadedExecutor*> execMap_;
    std::map<std::string, std::thread> subThMap_;

    // Data storage
    Recorder recorder_;
    Recorder recorderBk_;
    Recorder* recorderPtr_;
    Recorder* dumpPtr_;
    std::mutex recorderPtrLock_;// Prevent recorderPtr_ races.
    std::mutex dumpJSONLock_;// Protect dumpJSON() frequently called.
    
    SaveQueue<ImageMsg>* globalImgQue_;

    // Store ROS2 topic inforamtions
    TopicContainerPack topicContainerPack_;// Store topic name and message type.
    std::mutex topicContainerPackLock_;// Prevent topicContainerPack_ races.

    // Timer definitions
    vehicle_interfaces::Timer* monitorTimer_;// Scan topics on lan.
    vehicle_interfaces::Timer* sampleTimer_;// Sampling topic messages at fixed rate.
    vehicle_interfaces::Timer* dumpTimer_;// Dump messages buffer into files.
    vehicle_interfaces::Timer* recordTimer_;// Total execute time.
    std::mutex timerLock_;

    // Node behavior
    std::atomic<double> scanPeriod_ms_;
    std::atomic<double> samplingPeriod_ms_;
    std::atomic<double> dumpPeriod_ms_;
    std::atomic<double> recordDuration_ms_;

    // File manipulations
    fs::path dumpDir_;// Main data log directory.

    // Record node
    std::atomic<bool> exitF_;

private:

    void _paramsCallback(const rclcpp::Parameter param)// TODO: Not tested.
    {
        if (param.get_name() == "control_enable_monitor")
        {
            if (this->scanPeriod_ms_ <= 0)
                throw -1;

            if (this->monitorTimer_ == nullptr)
                this->monitorTimer_ = new vehicle_interfaces::Timer(this->scanPeriod_ms_, std::bind(&RecordNode::_monitorTimerCallback, this));

            if (param.as_bool())
                this->monitorTimer_->start();
            else
                this->monitorTimer_->stop();
        }
        else if (param.get_name() == "control_enable_record")
        {
            if (this->samplingPeriod_ms_ <= 0 || this->dumpPeriod_ms_ <= 0)
                throw -1;

            if (this->sampleTimer_ == nullptr)
                this->sampleTimer_ = new vehicle_interfaces::Timer(this->samplingPeriod_ms_, std::bind(&RecordNode::_samplingTimerCallback, this));
            if (this->dumpTimer_ == nullptr)
                this->dumpTimer_ = new vehicle_interfaces::Timer(this->dumpPeriod_ms_, std::bind(&RecordNode::_dumpTimerCallback, this));

            if (this->recordDuration_ms_ > 0 && this->recordTimer_ == nullptr)
                this->recordTimer_ = new vehicle_interfaces::Timer(this->recordDuration_ms_, std::bind(&RecordNode::_recordTimerCallback, this));

            if (param.as_bool())
                this->startRecord();
            else
                this->stopRecord();
        }
        else if (param.get_name() == "control_scan_period_ms")
        {
            if (param.as_double() <= 0)
                throw -1;

            this->scanPeriod_ms_ = param.as_double();
            if (this->monitorTimer_ == nullptr)
                this->monitorTimer_ = new vehicle_interfaces::Timer(this->scanPeriod_ms_, std::bind(&RecordNode::_monitorTimerCallback, this));
            else
                this->monitorTimer_->setInterval(this->scanPeriod_ms_);
        }
        else if (param.get_name() == "control_sampling_period_ms")
        {
            if (param.as_double() <= 0)
                throw -1;

            this->samplingPeriod_ms_ = param.as_double();
            if (this->sampleTimer_ == nullptr)
                this->sampleTimer_ = new vehicle_interfaces::Timer(this->samplingPeriod_ms_, std::bind(&RecordNode::_samplingTimerCallback, this));
            else
                this->sampleTimer_->setInterval(this->samplingPeriod_ms_);
        }
        else if (param.get_name() == "control_dump_period_s")
        {
            if (param.as_double() <= 0)
                throw -1;

            this->dumpPeriod_ms_ = param.as_double() * 1000.0;
            if (this->dumpTimer_ == nullptr)
                this->dumpTimer_ = new vehicle_interfaces::Timer(this->dumpPeriod_ms_, std::bind(&RecordNode::_dumpTimerCallback, this));
            else
                this->dumpTimer_->setInterval(this->dumpPeriod_ms_);
        }
        else if (param.get_name() == "control_record_duration_s")
        {
            if (param.as_double() <= 0)
                throw -1;

            this->recordDuration_ms_ = param.as_double() * 1000.0;
            if (this->recordTimer_ == nullptr)
                this->recordTimer_ = new vehicle_interfaces::Timer(this->recordDuration_ms_, std::bind(&RecordNode::_recordTimerCallback, this));
            else
                this->recordTimer_->setInterval(this->recordDuration_ms_);
        }
    }

    void _timeSyncCallback()
    {
        std::lock_guard<std::mutex> locker(this->topicContainerPackLock_);
        for (auto& [topicName, container] : this->topicContainerPack_)
            container.node->syncTime(this->getCorrectDuration(), this->getTimestampType());
    }

    void _monitorTimerCallback()
    {
        // Search topics
        auto map = this->get_topic_names_and_types();
        std::unique_lock<std::mutex> topicContainerLocker(this->topicContainerPackLock_, std::defer_lock);
        for (const auto& [topicName, msgTypeVec] : map)
        {
            auto msgTypeSplit = vehicle_interfaces::split(msgTypeVec.back(), "/");// Format: "<interfaces_pkg_name>/msg/<msg_type>".
            if (msgTypeSplit.size() != 3)
                continue;
            if (this->filterSet_.find(msgTypeSplit[0]) == this->filterSet_.end() && msgTypeSplit[1] != "msg")
                continue;

            topicContainerLocker.lock();
            if (this->topicContainerPack_.find(topicName) == this->topicContainerPack_.end())// New topic.
            {
                TopicContainer container;
                TopicInfo info;
                info.topicName = topicName;
                info.interface.packName = msgTypeSplit[0];
                info.interface.type = ROS2InterfaceType::MSG;
                info.interface.msgType = msgTypeSplit[2];
                container.info = info;
                container.occupyF = false;
                container.node = nullptr;
                this->topicContainerPack_[topicName] = container;
                RCLCPP_INFO(this->get_logger(), "[Scan] New topic [%s]: <%s, %s>", topicName.c_str(), container.info.interface.msgType.c_str(), container.occupyF ? "true" : "false");
            }
            topicContainerLocker.unlock();
        }

        topicContainerLocker.lock();
        for (auto& [topicName, container] : this->topicContainerPack_)
        {
            if (!container.occupyF)
            {
                auto splitTopicName = vehicle_interfaces::split(topicName, "/");
                std::string subNodeName = splitTopicName.back() + "_subnode";
                vehicle_interfaces::replace_all(subNodeName, "/", "_");

                if (container.info.interface.msgType == "Distance")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::Distance, vehicle_interfaces::msg::Distance> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "Environment")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::Environment, vehicle_interfaces::msg::Environment> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "GPS")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::GPS, vehicle_interfaces::msg::GPS> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "IDTable")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::IDTable, vehicle_interfaces::msg::IDTable> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "Image")
                    container.node = std::make_shared<SubSaveQueueNode<vehicle_interfaces::msg::Image, ImageMsg> >(subNodeName, container.info, this->globalImgQue_, this->dumpDir_, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "IMU")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::IMU, vehicle_interfaces::msg::IMU> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "MillitBrakeMotor")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::MillitBrakeMotor, vehicle_interfaces::msg::MillitBrakeMotor> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "MillitPowerMotor")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::MillitPowerMotor, vehicle_interfaces::msg::MillitPowerMotor> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "MotorAxle")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::MotorAxle, vehicle_interfaces::msg::MotorAxle> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "MotorSteering")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::MotorSteering, vehicle_interfaces::msg::MotorSteering> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "UPS")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::UPS, vehicle_interfaces::msg::UPS> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else if (container.info.interface.msgType == "WheelState")
                    container.node = std::make_shared<SubNode<vehicle_interfaces::msg::WheelState, vehicle_interfaces::msg::WheelState> >(subNodeName, container.info, this->params_->qosService, this->params_->qosDirPath);
                else
                    continue;
                
                if (!this->params_->enable_control)
                    container.node->setStoreFlag(true);

                container.node->syncTime(this->getCorrectDuration(), this->getTimestampType());
                this->execMap_[topicName] = new rclcpp::executors::SingleThreadedExecutor();
                this->execMap_[topicName]->add_node(container.node);
                this->subThMap_[topicName] = std::thread(SpinSubNodeExecutor, this->execMap_[topicName], container.node, topicName);
                
                RCLCPP_INFO(this->get_logger(), "[Scan] Subscribed [%s][%s]", topicName.c_str(), container.info.interface.msgType.c_str());
                container.occupyF = true;
            }
        }
        topicContainerLocker.unlock();
    }

    void _samplingTimerCallback()
    {
        std::lock_guard<std::mutex> topicContainerLocker(this->topicContainerPackLock_);
        std::lock_guard<std::mutex> recorderPtrLocker(this->recorderPtrLock_);

        std::string sampTsStr = std::to_string(this->getTimestamp().seconds());
        for (auto& [topicName, container] : topicContainerPack_)
        {
            if (!container.occupyF || container.node == nullptr)
                continue;
            if (!container.node->isInit())
                continue;

#ifndef LESS_COPY_METHOD
            // Non-refined solution. (Copy all data whether new or repetitive)
            bool newDataF = false;
            if (container.node->getData(this->recorderPtr_->msgQue, newDataF, true))
            {
                // Message data available
                this->recorderPtr_->recordMap[sampTsStr][topicName] = this->recorderPtr_->msgQue.size() - 1;// recordMap add idx corresponds to msgQue.
                container.msgBuf.first = this->recorderPtr_->msgQue.back();// Add tmp latest msg.
                container.msgBuf.second = this->recorderPtr_->msgQue.size() - 1;// Add tmp idx.
            }
#else
            // Less copy solution. (TODO: fix data disorder while record ptr switched. Fix random crash while dumping json.)
            if (this->recorderPtr_->msgQue.size() <= 0)// New msgQue or swapped.
            {
                bool newDataF = false;
                if (container.node->getData(this->recorderPtr_->msgQue, newDataF, true))// Force get msg data
                {
                    // Message data available
                    this->recorderPtr_->recordMap[sampTsStr][topicName] = 0;// recordMap add idx corresponds to msgQue.
                    container.msgBuf.first = this->recorderPtr_->msgQue.back();// Add tmp latest msg.
                    container.msgBuf.second = 0;// Add tmp idx.
                }
            }
            else// Normal situation
            {
                bool newDataF = false;
                if (container.node->getData(this->recorderPtr_->msgQue, newDataF, false))// msgQue pushed if newDataF
                {
                    // Message data available
                    if (newDataF)// New msg data arrived
                    {
                        // printf("%s new data\n", topicName.c_str());
                        this->recorderPtr_->recordMap[sampTsStr][topicName] = this->recorderPtr_->msgQue.size() - 1;// recordMap add idx corresponds to msgQue.
                        container.msgBuf.first = this->recorderPtr_->msgQue.back();// Add tmp latest msg.
                        container.msgBuf.second = this->recorderPtr_->msgQue.size() - 1;// Add tmp idx.
                        RCLCPP_WARN(this->get_logger(), "%s new data! [%ld]", topicName.c_str(), this->recorderPtr_->msgQue.size());
                    }
                    else
                    {
                        // printf("%s count: %d\n", topicName.c_str(), container.msgBuf.first.use_count());
                        if (container.msgBuf.first.use_count() < 2)// msgQue swapped. Copy tmp msg to current msgQue.
                        {
                            // this->recorderPtr_->msgQue.push_back(container.msgBuf.first);
                            // this->recorderPtr_->recordMap[sampTsStr][topicName] = this->recorderPtr_->msgQue.size() - 1;// recordMap add idx corresponds to msgQue.
                            // container.msgBuf.second = this->recorderPtr_->msgQue.size() - 1;
                            printf("%s recovered.\n", topicName.c_str());

                            // Grab msg data again while using force
                            if (container.node->getData(this->recorderPtr_->msgQue, newDataF, true))
                            {
                                // Message data available
                                this->recorderPtr_->recordMap[sampTsStr][topicName] = this->recorderPtr_->msgQue.size() - 1;// recordMap add idx corresponds to msgQue.
                                container.msgBuf.first = this->recorderPtr_->msgQue.back();// Add tmp latest msg.
                                container.msgBuf.second = this->recorderPtr_->msgQue.size() - 1;// Add tmp idx.
                            }
                        }
                        else if (container.msgBuf.second >= 0)// msgQue not swap.
                        {
                            this->recorderPtr_->recordMap[sampTsStr][topicName] = container.msgBuf.second;// recordMap add idx corresponds to msgQue.
                            RCLCPP_INFO(this->get_logger(), "%s linked. [%ld]", topicName.c_str(), container.msgBuf.second);
                        }
                    }
                }
            }
#endif
        }
    }

    void _dumpTimerCallback()
    {
        this->dumpJSON();
        std::string outStr = "Global image queue size: <";
        for (const auto& i : this->globalImgQue_->getSize())
            outStr = outStr + std::to_string(i) + ",";
        outStr.back() = '>';
        RCLCPP_INFO(this->get_logger(), "[Dump] %s", outStr.c_str());
        this->globalImgQue_->shrink_to_fit();
    }

    void _recordTimerCallback()
    {
        RCLCPP_WARN(this->get_logger(), "[RecordNode::_recordTimerCallback]");
        this->recordTimer_->stop();
        this->stopRecord();
    }

public:
    RecordNode(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        exitF_(false), 
        params_(params)
    {
        // Create data log directory
        {
            this->dumpDir_ = params->dump_path + "_" + std::to_string(this->get_clock()->now().seconds());
            char buf[256];
            sprintf(buf, "rm -rf %s && mkdir -p %s", this->dumpDir_.generic_string().c_str(), (this->dumpDir_ / "json").generic_string().c_str());
            const int dir_err = system(buf);
        }

        // Time sync callback
        this->addTimeSyncCallbackFunc(std::bind(&RecordNode::_timeSyncCallback, this));

        // Init recorder
        this->recorderPtr_ = &this->recorder_;
        this->dumpPtr_ = &this->recorderBk_;
        this->globalImgQue_ = new SaveQueue<ImageMsg>(params->img_threads, 100);

        // Add to set for filter element check.
        for (const auto& msgType : params->msg_filter)
            this->filterSet_.insert(msgType);

        // Set all timers to nullptr.
        this->monitorTimer_ = nullptr;
        this->sampleTimer_ = nullptr;
        this->dumpTimer_ = nullptr;
        this->recordTimer_ = nullptr;

        // Set parameters only if control enabled. Timers will be constructed while parameter service called.
        if (params->enable_control)
        {
            this->params_->addCallbackFunc(std::bind(&RecordNode::_paramsCallback, this, std::placeholders::_1));
            this->scanPeriod_ms_ = params->control_scan_period_ms;
            this->samplingPeriod_ms_ = params->control_sampling_period_ms;
            this->dumpPeriod_ms_ = params->control_dump_period_s * 1000.0;
            this->recordDuration_ms_ = params->control_record_duration_s * 1000.0;
            return;
        }

        this->scanPeriod_ms_ = params->scan_period_ms;
        this->samplingPeriod_ms_ = params->sampling_period_ms;
        this->dumpPeriod_ms_ = params->dump_period_s * 1000.0;
        this->recordDuration_ms_ = params->record_duration_s * 1000.0;

        if (this->scanPeriod_ms_ <= 0 || this->samplingPeriod_ms_ <= 0 || this->dumpPeriod_ms_ <= 0)
        {
            RCLCPP_WARN(this->get_logger(), "[RecordNode] Period of timer error: (scan: %.3lf > 0, sampling: %.3lf > 0, dump: %.3lf > 0). RecordNode will not function.", 
                        this->scanPeriod_ms_.load(), this->samplingPeriod_ms_.load(), this->dumpPeriod_ms_.load());
            return;
        }

        this->monitorTimer_ = new vehicle_interfaces::Timer(this->scanPeriod_ms_, std::bind(&RecordNode::_monitorTimerCallback, this));
        this->sampleTimer_ = new vehicle_interfaces::Timer(this->samplingPeriod_ms_, std::bind(&RecordNode::_samplingTimerCallback, this));
        this->dumpTimer_ = new vehicle_interfaces::Timer(this->dumpPeriod_ms_, std::bind(&RecordNode::_dumpTimerCallback, this));

        if (this->recordDuration_ms_ > 0)
            this->recordTimer_ = new vehicle_interfaces::Timer(this->recordDuration_ms_, std::bind(&RecordNode::_recordTimerCallback, this));

        this->monitorTimer_->start();
        this->startRecord();
    }

    ~RecordNode()
    {
        this->close();
    }

    void dumpJSON()
    {
        RCLCPP_INFO(this->get_logger(), "[Dump] Dumping json from buffer...");
        std::lock_guard<std::mutex> dumpLocker(this->dumpJSONLock_);
        std::unique_lock<std::mutex> recorderPtrLocker(this->recorderPtrLock_, std::defer_lock);

        // Swap ptr
        recorderPtrLocker.lock();
        std::swap(this->recorderPtr_, this->dumpPtr_);
        recorderPtrLocker.unlock();

        // Dump SubNodeMsg to JSON file
        double dumpTs = this->get_clock()->now().seconds();
        nlohmann::json recordJSON;

        auto st = std::chrono::steady_clock::now();// Calculate dumping spend time.
        for (const auto& [sampTs, topicIdxMap] : this->dumpPtr_->recordMap)
        {
            for (const auto& [topicName, idx] : topicIdxMap)
            {
                if (idx < 0)
                    continue;
                recordJSON[sampTs][topicName] = this->dumpPtr_->msgQue[idx]->dumpJSON();
            }
                
        }
        std::ofstream outFile(this->dumpDir_ / "json" / (std::to_string(dumpTs) + ".json"));
        outFile << recordJSON << std::endl;

        this->dumpPtr_->msgQue.clear();

        // Clear recorder
        this->dumpPtr_->recordMap.clear();

        RCLCPP_INFO(this->get_logger(), "[Dump] Dump %lf.json tooks %f ms", dumpTs, (std::chrono::steady_clock::now() - st).count() / 1000000.0);
    }

    // Get subscribed topic names and message types.
    std::map<std::string, std::string> getSubList()
    {
        std::lock_guard<std::mutex> topicContainerLocker(this->topicContainerPackLock_);
        std::map<std::string, std::string> ret;
        for (const auto& [topicName, container] : this->topicContainerPack_)
        {
            if (!container.occupyF || container.node == nullptr)
                continue;
            if (!container.node->isInit())
                continue;
            ret[topicName] = container.info.interface.msgType;
        }
        return ret;
    }

    template<typename T, typename U>
    bool getLatestData(const std::string& topicName, U& data)
    {
        std::lock_guard<std::mutex> topicContainerLocker(this->topicContainerPackLock_);
        if (this->topicContainerPack_.find(topicName) == this->topicContainerPack_.end())
            return false;
        if (!this->topicContainerPack_[topicName].occupyF || this->topicContainerPack_[topicName].node == nullptr)
            return false;
        if (!this->topicContainerPack_[topicName].node->isInit())
            return false;
        if (auto a = dynamic_cast<SubNode<T, U>*>(this->topicContainerPack_[topicName].node.get()))
            return a->getData(data);
        return false;
    }

    bool enableData(const std::string& topicName, bool flag)
    {
        std::lock_guard<std::mutex> topicContainerLocker(this->topicContainerPackLock_);
        if (this->topicContainerPack_.find(topicName) == this->topicContainerPack_.end())
            return false;
        if (!this->topicContainerPack_[topicName].occupyF || this->topicContainerPack_[topicName].node == nullptr)
            return false;
        if (!this->topicContainerPack_[topicName].node->isInit())
            return false;
        this->topicContainerPack_[topicName].node->setStoreFlag(flag);
        return true;
    }

    bool isDataEnable(const std::string& topicName)
    {
        std::lock_guard<std::mutex> topicContainerLocker(this->topicContainerPackLock_);
        if (this->topicContainerPack_.find(topicName) == this->topicContainerPack_.end())
            return false;
        if (!this->topicContainerPack_[topicName].occupyF || this->topicContainerPack_[topicName].node == nullptr)
            return false;
        if (!this->topicContainerPack_[topicName].node->isInit())
            return false;
        return this->topicContainerPack_[topicName].node->getStoreFlag();
    }

    void startRecord()
    {
        RCLCPP_WARN(this->get_logger(), "[RecordNode::startRecord]");
        std::unique_lock<std::mutex> topicContainerLocker(this->topicContainerPackLock_, std::defer_lock);
        std::lock_guard<std::mutex> locker(this->timerLock_);

        topicContainerLocker.lock();
        for (auto& [topicName, container] : this->topicContainerPack_)
            container.node->setStoreFlag(true);
        topicContainerLocker.unlock();

        this->sampleTimer_->start();
        this->dumpTimer_->start();
        if (this->recordTimer_ != nullptr)
            this->recordTimer_->start();
    }

    void stopRecord()
    {
        RCLCPP_WARN(this->get_logger(), "[RecordNode::stopRecord]");
        std::unique_lock<std::mutex> topicContainerLocker(this->topicContainerPackLock_, std::defer_lock);
        std::lock_guard<std::mutex> locker(this->timerLock_);

        topicContainerLocker.lock();
        for (auto& [topicName, container] : this->topicContainerPack_)
            container.node->setStoreFlag(false);
        topicContainerLocker.unlock();

        RCLCPP_WARN(this->get_logger(), "[RecordNode::stopRecord] setStoreFlag(false)");

        this->sampleTimer_->stop();
        this->dumpTimer_->stop();
        if (this->recordTimer_ != nullptr)
            this->recordTimer_->stop();
        this->dumpJSON();// Dump rest of files in memory to storage.
    }

    void close()
    {
        std::lock_guard<std::mutex> locker(this->timerLock_);
        if (this->exitF_)// Already closed
            return;

        RCLCPP_WARN(this->get_logger(), "[RecordNode::close]");

        if (this->monitorTimer_ != nullptr)
        {
            this->monitorTimer_->destroy();// Timer destroys while callback function returned.
            RCLCPP_WARN(this->get_logger(), "[RecordNode::close] monitorTimer_ destroyed.");
        }

        if (this->sampleTimer_ != nullptr)
        {
            this->sampleTimer_->destroy();// Stop recording data from topics.
            RCLCPP_WARN(this->get_logger(), "[RecordNode::close] sampleTimer_ destroyed.");
        }

        if (this->dumpTimer_ != nullptr)
        {
            this->dumpTimer_->destroy();// Stop saving json files.
            RCLCPP_WARN(this->get_logger(), "[RecordNode::close] dumpTimer_ destroyed.");
        }

        if (this->recordTimer_ != nullptr)
        {
            this->recordTimer_->destroy();// Stop saving json files.
            RCLCPP_WARN(this->get_logger(), "[RecordNode::close] recordTimer_ destroyed.");
        }

        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Dumping files...");
        this->dumpJSON();// saving rest of data in memories to json file.

        // Stop running subscriber nodes
        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Closing subscriber...");
        for (auto &i : this->execMap_)
            i.second->cancel();
        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Closing subscriber done.");

        // Join subscriber threads
        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Subscriber threads join...");
        for (auto &i : this->subThMap_)
            i.second.join();
        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Subscriber threads join done.");

        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] Deleting SaveQueue...");
        delete this->globalImgQue_;// Data in queues will be saved while destructor finished.
        RCLCPP_INFO(this->get_logger(), "[RecordNode::close] SaveQueue deleted.");

        this->exitF_ = true;// Set exit flag to true to prevent multiple close function calling.
    }

    bool isExit() const { return this->exitF_; }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("dataserver_params_node");// params cannot be manipulated outside recordNode.
    auto recordNode = std::make_shared<RecordNode>(params);
    rclcpp::executors::SingleThreadedExecutor* executor = new rclcpp::executors::SingleThreadedExecutor();
    executor->add_node(recordNode);
    std::thread recordNodeTh(SpinExecutor, executor, "recordNodeTh");

    executor->cancel();
    recordNodeTh.join();
    recordNode->close();
    delete executor;
    RCLCPP_INFO(rclcpp::get_logger("Main"), "Done.");
    rclcpp::shutdown();
    return 0;
}