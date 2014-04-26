/**
 * @file /include/objectInfo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef objectInfo_QNODE_HPP_
#define objectInfo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <QThread>
#include <QStringListModel>
using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace objectInfo {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void tiltAngleCb(const std_msgs::Float64ConstPtr& tilt);
	/*********************
	** Logging
	**********************/
    enum LogLevel {Debug,Info,  Warn, Error, Fatal };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    float currentTiltAngle;
    float tiltAngle;

signals:
	void loggingUpdated();
    void rosShutdown();
    void angleChanged();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher  pubTiltAngle;
    ros::Subscriber subTiltAngle;

    std_msgs::Float64 newTiltAngle;
    QStringListModel logging_model;
};

}  // namespace objectInfo

#endif /* objectInfo_QNODE_HPP_ */
