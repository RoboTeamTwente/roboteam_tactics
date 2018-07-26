package nodewrapper
// #cgo pkg-config: roboteam_tactics roboteam_utils roboteam_msgs roscpp roslib
// #include "NodeWrapper.hpp"
// #include <stdlib.h>
import "C"

import (
  "fmt"
  "bytes"
  "unsafe"
  "errors"
  "sync"
  "roboteam_msgs"
  bt "behaviortree"
)

var GlobalCrapLock = new(sync.Mutex)

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

  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  return C.NodeNew(CClassName, CNodeName, C.int(buflen), str);
}

func (n *_Ctype_struct_CNode) Initiate() {
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.NodeInitiate(n)
}

func (n *_Ctype_struct_CNode) Update(state interface {}, messages []interface {}) []interface {} {
  // TODO pass world state here
  // maybe capture ROS messages?
  // DEFINITELY set up blackboard correctly
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.NodeUpdate(n)
  return messages
}

func (n *_Ctype_struct_CNode) Terminate() {
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.NodeTerminate(n)
}

func (n *_Ctype_struct_CNode) GetStatus() bt.Status {
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
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
    "AimAt",
    "AimingAtOpponent",
    "Anouk_BallPlacementThemPlay",
    "Anouk_BallPlacementUsPlay",
    "Anouk_MultipleDefendersPlay",
    "Anouk_PenaltyKeeper",
    "Anouk_PrepareDirectThem",
    "Anouk_StopPlay",
    "BallOnOurSide",
    "BallOnTheirSide",
    "BallPlacementTest",
    "BallPlacementUsPlay",
    "Block",
    "Bob_ChipoffAtGoalPlay",
    "Bob_KickoffWithRunPlay",
    "CanInterceptBallDuel",
    "CanKeeperMove",
    "CanSeePoint",
    "CanSeeRobot",
    "CanSeeTheirGoal",
    "Chip",
    "DebugTrace",
    "DefendPenalty",
    "DistanceXToY",
    "Dribble",
    "Emiel_IndirectUsPlay",
    "Emiel_PrepareKickoffThem",
    "Emiel_PrepareKickoffUs",
    "Emiel_PreparePenaltyThem",
    "Emiel_PreparePenaltyUs",
    "FormationPlay",
    "Freeze",
    "GetBall",
    "GetBallTest",
    "GoToPos",
    "HaltTactic",
    "Harass",
    "IHaveBall",
    "IsBallInGoal",
    "IsBallMovingTowardsRobot",
    "IsInDefenseArea",
    "IsInZone",
    "IsRefCommand",
    "IsRefStage",
    "IsRobotClosestToBall",
    "Jelle_DemoPlay",
    "Jelle_IndependentAttackersPlay",
    "Jim_GetBallPlay",
    "Jim_IndependentAttackersPlay",
    "Jim_IndirectGetBallPlay",
    "Jim_KickOffDefense",
    "Jim_MultipleDefendersPlay",
    "Jim_MultipleDefendersPlayOld",
    "Jim_MultipleStrikersPlay",
    "Jim_PenaltyPlay",
    "Jim_TimeOut",
    "KeeperV2",
    "KeepPosition",
    "Kick",
    "KickoffDefensePlay",
    "KickoffUsTactic",
    "KickPenalty",
    "MirrorOpponent",
    "NaiveBlockGoal",
    "ParamCheck",
    "ParamSet",
    "PassToMe",
    "PenaltyThemPlay",
    "PenaltyUsPlay",
    "Positioning",
    "PrepareKeeperPenaltyThem",
    "PrepareKickoffUsTactic",
    "Presentation_SetPositions",
    "Push",
    "Qualification1v1Tactic",
    "ReceiveBall",
    "RotateAroundPoint",
    "rtt_bob/NaiveGetBallWithSupportPlay",
    "rtt_bob/SingleKeeperPlay",
    "SelectStarAttackShooter",
    "ShootAtGoalV2",
    "SimpleDefender",
    "SimpleGoalDefender",
    "SoloAttacker2Tactic",
    "SoloAttackerTactic",
    "SoloDefenderTactic",
    "StandByTactic",
    "StandFree",
    "StopPlay",
    "TargetPracticeTactic",
    "TheyHaveBall",
    "ThrowinPlay",
    "Twirl",
    "TwirlPlay",
    "TwoAttackersCoolTactic",
    "TwoAttackersTactic",
  }
  for _, name := range nodeNames {
    myName := name
    bt.NodeTypeRegister[myName] = func(root bt.ProjectNode, nodes map[string]bt.ProjectNode, props map[string]interface{})(n bt.Node, err error) {
      defer func() {
        cerr := recover()
        if cerr != nil {
          err = errors.New("C++ freaked out. Fuck C++")
        }
      }()
      // merge the "blackboards"
      myProps := make(map[string]interface{})
      for prop, val := range root.Properties {
        myProps[prop] = val
      }
      for prop, val := range props {
        myProps[prop] = val
      }
      return NewCNode(myName, root.Title, myProps), nil
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
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.SetWorld(C.int(buflen), str)
}

func SetField(msg *roboteam_msgs.GeometryData) {
  var buf bytes.Buffer
  msg.Field.Serialize(&buf)
  buflen := buf.Len()
	//fmt.Printf("Received: %d\n", buflen)
  str := C.CString(buf.String())
  defer C.free(unsafe.Pointer(str))
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.SetField(C.int(buflen), str)
}

func SetRef(msg *roboteam_msgs.RefereeData) {
  var buf bytes.Buffer
  msg.Serialize(&buf)
  buflen := buf.Len()
	//fmt.Printf("Received: %d\n", buflen)
  str := C.CString(buf.String())
  defer C.free(unsafe.Pointer(str))
  GlobalCrapLock.Lock()
  defer GlobalCrapLock.Unlock()
  C.SetRef(C.int(buflen), str)
}
