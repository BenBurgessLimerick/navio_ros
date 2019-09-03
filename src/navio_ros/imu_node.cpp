
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"


#include <sys/time.h>

#include <Common/InertialSensor.h>
#include "AHRS.hpp"
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/Util.h>


std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}

//============================== Main loop ====================================

void imuLoop(AHRS* ahrs, ros::Publisher publisher)
{
    sensor_msgs::Imu msg;
    //fill in header here

    // Orientation data
    float roll, pitch, yaw;

    struct timeval tv;
    float dt;
    // Timing data

    static float maxdt;
    static float mindt = 0.01;
    static float dtsumm = 0;
    static int isFirst = 1;
    static unsigned long previoustime, currenttime;


    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    ahrs->updateIMU(dt);

    //------------------------ Read Euler angles ------------------------------

    ahrs->getEuler(&roll, &pitch, &yaw);

    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------

    dtsumm += dt;
    if(dtsumm > 0.05)
    {
        // Console output
        printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));

        dtsumm = 0;
    }

    msg.orientation.x = ahrs->getX();
    msg.orientation.y = ahrs->getY();
    msg.orientation.z = ahrs->getZ();
    msg.orientation.w = ahrs->getW();
    //TODO: Covariances

    float gx, gy, gz;
    ahrs->read_gyroscope(&gx, &gy, &gz);
    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;
    //TODO: Covariances


    float ax, ay, az;
    ahrs->read_accelerometer(&ax, &ay, &az);
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
    //TODO: Covariances
    publisher.publish(msg);
}

//=============================================================================

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }

    std::string sensor_name = "mpu";

    if (sensor_name.empty())
        return EXIT_FAILURE;

    auto imu = get_inertial_sensor(sensor_name);

    if (!imu) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!imu->probe()) {
        printf("Sensor not enable\n");
        return EXIT_FAILURE;
    }

	ros::init(argc, argv, "navio_imu_" + sensor_name +  "_node");
	ros::NodeHandle n;
	ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("navio_imu_" + sensor_name, 10);
    ros::Rate loop_rate(100);


    auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };

    //-------------------- Setup gyroscope offset -----------------------------

    ahrs->setGyroOffset();

    int count = 0;

    while(ros::ok())
        imuLoop(ahrs.get(), imu_publisher);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
        
}
