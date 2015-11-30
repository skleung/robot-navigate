import Queue
import time

class State:
  def __init__(self):
    self.name = ''
    self.transitions = {}

  # Takes a transition name and assigns it to a callback
  # transition: event name, callback: action/behavior, toState: final state after action/behavior
  def add_transition(self, transition, callback, toState):
    self.transitions[transition] = (callback, toState)


class StateMachine:
  def __init__(self, robot, joystick):
    self.states = {}
    self.currentState = None
    self.robot = robot
    self.numTags = 0
    self.joystick = joystick
    self.speed = 30
    self.queue = Queue.Queue()

    # ---- NOT IT ----
    # walk
    walk_state = self.add_state("Walk")
    self.set_start("Walk")

    walk_state.add_transition("got tagged", self.turnOn, "It Walk")
    walk_state.add_transition("tagged", self.turnOff, "Walk")
    walk_state.add_transition("obj right", self.move_left, "Walk Left")
    walk_state.add_transition("obj left", self.move_right, "Walk Right")
    walk_state.add_transition("obj ahead", self.move_right, "Walk Right")
    walk_state.add_transition("clear", self.move_up, "Walk")
    walk_state.add_transition("floor left", self.move_right, "Walk Edge Left")
    walk_state.add_transition("floor right", self.move_left, "Walk Edge Right")
    walk_state.add_transition("stop", self.stop_move, "Stop")
    walk_state.add_transition("victory", self.victory_dance, "Victory")


    # walk edge left
    walk_edge_left_state = self.add_state("Walk Edge Left")
    walk_edge_left_state.add_transition("floor left", self.move_right, "Walk Edge Left")
    walk_edge_left_state.add_transition("floor right", self.move_right, "Walk Edge Left")
    walk_edge_left_state.add_transition("floor clear", self.delay_move_up, "Walk")
    walk_edge_left_state.add_transition("got tagged", self.turnOn, "It Walk")
    walk_edge_left_state.add_transition("tagged", self.turnOff, "Walk")
    walk_edge_left_state.add_transition("stop", self.stop_move, "Stop")
    walk_edge_left_state.add_transition("victory", self.victory_dance, "Victory")


    # walk edge right
    walk_edge_right_state = self.add_state("Walk Edge Right")
    walk_edge_right_state.add_transition("floor left", self.move_left, "Walk Edge Right")
    walk_edge_right_state.add_transition("floor right", self.move_left, "Walk Edge Right")
    walk_edge_right_state.add_transition("floor clear", self.delay_move_up, "Walk")
    walk_edge_right_state.add_transition("got tagged", self.turnOn, "It Walk")
    walk_edge_right_state.add_transition("tagged", self.turnOff, "Walk")
    walk_edge_right_state.add_transition("stop", self.stop_move, "Stop")
    walk_edge_right_state.add_transition("victory", self.victory_dance, "Victory")

    # walk left
    walk_left_state = self.add_state("Walk Left")
    walk_left_state.add_transition("obj right", self.move_left, "Walk Left")
    walk_left_state.add_transition("obj left", self.move_right, "Walk Right")
    walk_left_state.add_transition("obj ahead", self.move_left, "Walk Left")
    walk_left_state.add_transition("clear", self.move_up, "Walk")
    walk_left_state.add_transition("floor left", self.move_right, "Walk Edge Left")
    walk_left_state.add_transition("floor right", self.move_left, "Walk Edge Right")
    walk_left_state.add_transition("got tagged", self.turnOn, "It Walk")
    walk_left_state.add_transition("tagged", self.turnOff, "Walk")
    walk_left_state.add_transition("stop", self.stop_move, "Stop")
    walk_left_state.add_transition("victory", self.victory_dance, "Victory")

    # walk right
    walk_right_state = self.add_state("Walk Right")
    walk_right_state.add_transition("obj right", self.move_left, "Walk Left")
    walk_right_state.add_transition("obj left", self.move_right, "Walk Right")
    walk_right_state.add_transition("obj ahead", self.move_left, "Walk Right")
    walk_right_state.add_transition("clear", self.move_up, "Walk")
    walk_right_state.add_transition("floor left", self.move_right, "Walk Edge Left")
    walk_right_state.add_transition("floor right", self.move_left, "Walk Edge Right")
    walk_right_state.add_transition("got tagged", self.turnOn, "It Walk")
    walk_right_state.add_transition("tagged", self.turnOff, "Walk")
    walk_right_state.add_transition("stop", self.stop_move, "Stop")
    walk_right_state.add_transition("victory", self.victory_dance, "Victory")


    # ---- IT ----
    # walk
    it_walk_state = self.add_state("It Walk")
    it_walk_state.add_transition("tagged", self.turnOff, "Walk")
    it_walk_state.add_transition("got tagged", self.turnOn, "It Walk")
    it_walk_state.add_transition("obj right", self.move_right, "It Walk Right")
    it_walk_state.add_transition("obj left", self.move_left, "It Walk Left")
    it_walk_state.add_transition("obj ahead", self.move_up, "It Ram")
    it_walk_state.add_transition("clear", self.move_up, "It Walk")
    it_walk_state.add_transition("floor left", self.move_right, "It Walk Edge Left")
    it_walk_state.add_transition("floor right", self.move_left, "It Walk Edge Right")
    it_walk_state.add_transition("stop", self.stop_move, "Stop")
    it_walk_state.add_transition("victory", self.victory_dance, "Victory")

    # it ram
    it_ram = self.add_state("It Ram")
    it_ram.add_transition("clear", self.move_up, "It Walk")
    it_ram.add_transition("tagged", self.turnOff, "Walk")
    it_ram.add_transition("got tagged", self.turnOn, "It Walk")
    it_ram.add_transition("floor left", self.move_right, "It Walk Edge Left")
    it_ram.add_transition("floor right", self.move_left, "It Walk Edge Right")
    it_ram.add_transition("stop", self.stop_move, "Stop")
    it_ram.add_transition("victory", self.victory_dance, "Victory")

    # walk edge left
    it_walk_edge_left_state = self.add_state("It Walk Edge Left")
    it_walk_edge_left_state.add_transition("floor left", self.move_right, "It Walk Edge Left")
    it_walk_edge_left_state.add_transition("floor right", self.move_right, "It Walk Edge Left")
    it_walk_edge_left_state.add_transition("floor clear", self.delay_move_up, "It Walk")
    it_walk_edge_left_state.add_transition("tagged", self.turnOff, "Walk")
    it_walk_edge_left_state.add_transition("got tagged", self.turnOn, "It Walk")
    it_walk_edge_left_state.add_transition("stop", self.stop_move, "Stop")
    it_walk_edge_left_state.add_transition("victory", self.victory_dance, "Victory")

    # walk edge right
    it_walk_edge_right_state = self.add_state("It Walk Edge Right")
    it_walk_edge_right_state.add_transition("floor left", self.move_left, "It Walk Edge Right")
    it_walk_edge_right_state.add_transition("floor right", self.move_left, "It Walk Edge Right")
    it_walk_edge_right_state.add_transition("floor clear", self.delay_move_up, "It Walk")
    it_walk_edge_right_state.add_transition("tagged", self.turnOff, "Walk")
    it_walk_edge_right_state.add_transition("got tagged", self.turnOn, "It Walk")
    it_walk_edge_right_state.add_transition("stop", self.stop_move, "Stop")
    it_walk_edge_right_state.add_transition("victory", self.victory_dance, "Victory")

    # walk left
    it_walk_left_state = self.add_state("It Walk Left")
    it_walk_left_state.add_transition("obj right", self.move_right, "It Walk Right")
    it_walk_left_state.add_transition("obj left", self.move_left, "It Walk Left")
    it_walk_left_state.add_transition("obj ahead", self.move_up, "It Walk")
    it_walk_left_state.add_transition("clear", self.move_up, "It Walk")
    it_walk_left_state.add_transition("floor left", self.move_right, "It Walk Edge Left")
    it_walk_left_state.add_transition("floor right", self.move_left, "It Walk Edge Right")
    it_walk_left_state.add_transition("tagged", self.turnOff, "Walk")
    it_walk_left_state.add_transition("got tagged", self.turnOn, "It Walk")
    it_walk_left_state.add_transition("stop", self.stop_move, "Stop")
    it_walk_left_state.add_transition("victory", self.victory_dance, "Victory")

    # walk right
    it_walk_right_state = self.add_state("It Walk Right")
    it_walk_right_state.add_transition("obj right", self.move_right, "It Walk Right")
    it_walk_right_state.add_transition("obj left", self.move_left, "It Walk Left")
    it_walk_right_state.add_transition("obj ahead", self.move_up, "It Walk")
    it_walk_right_state.add_transition("clear", self.move_up, "It Walk")
    it_walk_right_state.add_transition("floor left", self.move_right, "It Walk Edge Left")
    it_walk_right_state.add_transition("floor right", self.move_left, "It Walk Edge Right")
    it_walk_right_state.add_transition("tagged", self.turnOff, "Walk")
    it_walk_right_state.add_transition("got tagged", self.turnOn, "It Walk")
    it_walk_right_state.add_transition("stop", self.stop_move, "Stop")
    it_walk_right_state.add_transition("victory", self.victory_dance, "Victory")

    # final states
    # stop
    stop = self.add_state("Stop")
    # victory
    victory = self.add_state("Victory")

  def victory_dance(self):
    self.joystick.speed = 90
    self.move_left()
    for i in range(7):
        self.robot.set_led(0,i)
        self.robot.set_led(1,i)
        self.joystick.play_sound()
        time.sleep(0.5)
    self.stop_move()

  def add_state(self, name):
    a_state = State()
    a_state.name = name
    self.states[name] = a_state
    return a_state

  def set_start(self, name):
    self.currentState = name

  # behaviors
  def set_light(self, on):
    if on:
      self.robot.set_led(0,4)
      self.robot.set_led(1,4)
    else:
      self.robot.set_led(0,1)
      self.robot.set_led(1,1)

  # set not it state
  def turnOff(self):
    print "turning off"
    self.set_light(False)
    self.speed = 30
    self.move_up()

  # set it state
  def turnOn(self):
    print "turning on"
    self.set_light(True)
    self.speed = 70
    self.numTags += 1
    self.move_up()

  def delay_move_up(self):
    time.sleep(1)
    self.queue.queue.clear()
    self.move_up()

  def move_up(self):
    self.joystick.speed = self.speed
    self.joystick.move_up()

  def move_down(self):
    self.joystick.speed = self.speed
    self.joystick.move_down()

  def move_left(self):
    self.joystick.speed = self.speed
    self.joystick.move_left()

  def move_right(self):
    self.joystick.speed = self.speed
    self.joystick.move_right()

  def stop_move(self):
    self.joystick.stop_move()

  def run(self):
    self.move_up()
    while(True):
      state = self.states[self.currentState]
      if self.queue.empty():
        time.sleep(0.05)
        continue
      transitionName = self.queue.get()
      if transitionName in state.transitions:
        # print self.currentState, "-->", state.transitions[transitionName][1]
        state.transitions[transitionName][0]()
        self.currentState = state.transitions[transitionName][1]
