package main

import (
  "os"
  "fmt"
  "time"
  "log"
  "path/filepath"
  "github.com/cnord/rosgo/ros"
  "roboteam_msgs"
  nw "roboteam_tactics/nodewrapper"
  bt "behaviortree"
)

var worldChan = make(chan *roboteam_msgs.World)
func worldCallback(msg *roboteam_msgs.World) {
  nw.SetWorld(msg)
  worldChan <- msg
}

var fieldChan = make(chan *roboteam_msgs.GeometryData)
func fieldCallback(msg *roboteam_msgs.GeometryData) {
  nw.SetField(msg)
  fieldChan <- msg
}

var refChan = make(chan *roboteam_msgs.RefereeData)
func refCallback(msg *roboteam_msgs.RefereeData) {
  nw.SetRef(msg)
  refChan <- msg
}

type Universe struct {
  // Implicit dependency on ROS
  // New Go code should probably not use these
  World *roboteam_msgs.World
  Field *roboteam_msgs.GeometryData
  Ref   *roboteam_msgs.RefereeData
}

func universeCollector(interval time.Duration, uniChan chan<- Universe) {
  ticker := time.NewTicker(interval)
  var universe Universe
  for {
    select {
    case world := <-worldChan:
      universe.World = world
    case field := <-fieldChan:
      universe.Field = field
    case ref := <-refChan:
      universe.Ref = ref
    case <-ticker.C:
      uniChan <- universe
    }
  }
}

// TODO StrategyUniverse

type Role struct {
  TickChan chan Universe
  Id int
  Tree string
  Props map[string]interface{}
}

func blackboardToMap(bbmsg *roboteam_msgs.Blackboard) map[string]interface{} {
  bbmap := make(map[string]interface{})
  for _, e := range bbmsg.Bools {
    bbmap[e.Name] = e.Value
  }
  for _, e := range bbmsg.Ints {
    bbmap[e.Name] = e.Value
  }
  for _, e := range bbmsg.Doubles {
    bbmap[e.Name] = e.Value
  }
  for _, e := range bbmsg.Strings {
    bbmap[e.Name] = e.Value
  }
  return bbmap
}

var roleChan = make(chan *roboteam_msgs.RoleDirective)
func roleDirectiveCallback(msg *roboteam_msgs.RoleDirective) {
  roleChan <- msg
}

func roleNode(universeChan <-chan Universe) {
  roles := make(map[int32]Role)
  completion := make(chan int)
  for {
    select {
    case roleMsg := <-roleChan:
      role, ok := roles[roleMsg.RobotId]
      if ok {
        close(role.TickChan)
      } else {
        role = Role{}
        roles[roleMsg.RobotId] = role
      }
      role.Props = blackboardToMap(&roleMsg.Blackboard)
      role.Tree = roleMsg.Tree
      role.Id = int(roleMsg.RobotId)
      role.TickChan = make(chan Universe)
      // TODO pass token
      go roleRunner(role, completion)
    case universe := <-universeChan:
      for _, role := range roles {
        select {
        case role.TickChan <- universe:
        default:
          log.Printf("Role %s did not respond to tick", role.Tree)
        }
      }
    case robotId := <-completion:
      role := roles[int32(robotId)]
      close(role.TickChan)
      // TODO send feedback
      delete(roles, int32(robotId))
    }
  }
}

func roleRunner(role Role, completion chan<- int) {
  tree := trees[role.Tree]
  for universe := range role.TickChan {
    fmt.Println(role.Tree, role.Id)
    status, _ := bt.Tick(tree, universe, nil)
    // TODO send messages
    if status != bt.Running {
      break
    }
  }
  tree.Terminate()
  completion <- role.Id
}

func check(e error) {
    if e != nil {
        panic(e)
    }
}

var trees = make(map[string]bt.Node)

func init() {
  files, err := filepath.Glob("trees/*.b3")
  check(err)
  for _, fname := range files {
    f, err := os.Open(fname)
    fmt.Println(fname)
    check(err)
    defer f.Close()
    pr, err := bt.ReadProject(f)
    check(err)
    bt.MakeTrees(pr, trees)
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
	node.NewSubscriber("/vision_refbox", roboteam_msgs.MsgRefereeData, nw.SetRef)

  universeChan := make(chan Universe)
  go universeCollector(1/60*time.Second, universeChan)
  go roleNode(universeChan)

  node.Spin()
}
