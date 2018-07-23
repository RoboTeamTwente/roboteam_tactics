package main

import (
  "os"
  "fmt"
  "github.com/cnord/rosgo/ros"
  "roboteam_msgs"
  nw "roboteam_tactics/nodewrapper"
  bt "behaviortree"
)

func worldCallback(msg *roboteam_msgs.World) {
  nw.SetWorld(msg)

  bt.Tick(btNode)
  //fmt.Println(status)
}

func check(e error) {
    if e != nil {
        panic(e)
    }
}

var btNode bt.Node

func main() {
  fmt.Println("Starting")
	node := ros.NewNode("/amazing_go")
	defer node.Shutdown()
	//node.Logger().SetSeverity(ros.LogLevelDebug)
	node.NewSubscriber("/world_state", roboteam_msgs.MsgWorld, worldCallback)
	node.NewSubscriber("/vision_geometry", roboteam_msgs.MsgGeometryData, nw.SetField)

  f, err := os.Open("/home/pepijn/code/roboteam/workspace/src/roboteam_tactics/src/trees/projects/rtt_pepijn.b3")
  check(err)
  defer f.Close()
  pr, err := bt.ReadProject(f)
  check(err)
  trees := bt.MakeTrees(pr)
  btNode = trees[0]

  for btNode.Status() != bt.Success {
    node.SpinOnce()
  }
}
