package nodewrapper
// #cgo CFLAGS: -I/home/pepijn/code/roboteam/ros_catkin_ws/install_isolated/include/
// #cgo CFLAGS: -I/home/pepijn/code/roboteam/workspace/devel/include
// #cgo CFLAGS: -I/home/pepijn/code/roboteam/workspace/src/roboteam_utils/include
// #cgo CFLAGS: -I/home/pepijn/code/roboteam/workspace/src/roboteam_tactics/include
// #cgo CXXFLAGS: -I/home/pepijn/code/roboteam/ros_catkin_ws/install_isolated/include/
// #cgo CXXFLAGS: -I/home/pepijn/code/roboteam/workspace/devel/include
// #cgo CXXFLAGS: -I/home/pepijn/code/roboteam/workspace/src/roboteam_utils/include
// #cgo CXXFLAGS: -I/home/pepijn/code/roboteam/workspace/src/roboteam_tactics/include
// #cgo LDFLAGS: -L/home/pepijn/code/roboteam/workspace/devel/lib
// #cgo LDFLAGS: -L/home/pepijn/code/roboteam/ros_catkin_ws/install_isolated/lib
// #cgo LDFLAGS: -lSkill -lbt -lroboteam_utils -lroscpp -lroscpp_serialization -lroslib
// #include "NodeWrapper.hpp"
// #include <stdlib.h>
import "C"

import (
  "fmt"
  "bytes"
  "unsafe"
  "roboteam_msgs"
  bt "behaviortree"
)

func toBlackboard(props map[string]interface{}) *roboteam_msgs.Blackboard {
  bb := new(roboteam_msgs.Blackboard)
  for key, value := range props {
    switch val := value.(type) {
    case float64:
      bb.Doubles = append(bb.Doubles, roboteam_msgs.Float64Entry{key, val})
    case bool:
      bb.Bools = append(bb.Bools, roboteam_msgs.BoolEntry{key, val})
    case int32:
      bb.Ints = append(bb.Ints, roboteam_msgs.Int32Entry{key, val})
    case string:
      bb.Strings = append(bb.Strings, roboteam_msgs.StringEntry{key, val})
    default:
      fmt.Printf("Cannot set key %s of type %t", key, value)
    }
  }
  return bb
}

func NewCNode(className string, nodeName string, props map[string]interface{}) *_Ctype_struct_CNode {
  fmt.Println(className, nodeName)
  CClassName := C.CString(className)
  CNodeName := C.CString(nodeName)
  //defer C.free(unsafe.Pointer(str))

  msg := toBlackboard(props)
  var buf bytes.Buffer
  msg.Serialize(&buf)
  buflen := buf.Len()
  str := C.CString(buf.String())
  defer C.free(unsafe.Pointer(str))

  return C.NodeNew(CClassName, CNodeName, C.int(buflen), str);
}

func (n *_Ctype_struct_CNode) Initiate() {
  C.NodeInitiate(n)
}

func (n *_Ctype_struct_CNode) Update() {
  C.NodeUpdate(n)
}

func (n *_Ctype_struct_CNode) Terminate() {
  C.NodeTerminate(n)
}

func (n *_Ctype_struct_CNode) Status() bt.Status {
  switch C.NodeStatus(n) {
  case 1:
    return bt.Success
  case 2:
    return bt.Failure
  case 3:
    return bt.Running
  default:
    return bt.Failure
  }
}

func init() {
  C.RosInit()
  // C++ nodes
  nodeNames := []string{
    "GoToPos",
    "Freeze",
  }
  for _, name := range nodeNames {
    myName := name
    bt.NodeTypeRegister[myName] = func(root bt.ProjectNode, nodes map[string]bt.ProjectNode)bt.Node {
      return NewCNode(myName, root.Title, root.Properties)
    }
  }
}

func SetWorld(msg *roboteam_msgs.World) {
  var buf bytes.Buffer
  msg.Serialize(&buf)
  buflen := buf.Len()
	//fmt.Printf("Received: %d\n", buflen)
  str := C.CString(buf.String())
  defer C.free(unsafe.Pointer(str))
  C.SetWorld(C.int(buflen), str)
}

func SetField(msg *roboteam_msgs.GeometryData) {
  var buf bytes.Buffer
  msg.Field.Serialize(&buf)
  buflen := buf.Len()
	//fmt.Printf("Received: %d\n", buflen)
  str := C.CString(buf.String())
  defer C.free(unsafe.Pointer(str))
  C.SetField(C.int(buflen), str)
}
