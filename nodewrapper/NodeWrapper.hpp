#ifdef __cplusplus
#include "ros/ros.h"
#include "roboteam_tactics/bt/Node.hpp"
#include <memory>
struct CNode {
    std::shared_ptr<bt::Node> CxxNode;
};
#else
typedef struct CNode CNode;
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
    void RosInit();
    void RosShutdown();

    void SetWorld(int len, char* data);
    void SetField(int len, char* data);
    void SetRef(int len, char* data);

    CNode* NodeNew(char* className, char* nodeName, int len, char* bbdata);
    void NodeRelease(CNode* t);
    void NodeInitiate(CNode* t);
    void NodeUpdate(CNode* t);
    void NodeTerminate(CNode* t);
    int NodeStatus(CNode* t);
    void DeleteNode(CNode* t);

#ifdef __cplusplus
}
#endif
