#include "autonomous_navigation/nsclControl.h"
#include "autonomous_navigation/nsclStatus.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "NSCLNode");

    NSCLControl mc;
    NSCLStatus ms;

    boost::thread* thr = new boost::thread(boost::bind(&NSCLControl::doAutoNav, &mc));
    boost::thread* thr2 = new boost::thread(boost::bind(&NSCLStatus::publishNSCLStatus, &ms));
   
    thr->join();     
    thr2->join();        
    
}
