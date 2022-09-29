""" Interrupt Task """

import weakref
from rasberry_coordination.task_management.__init__ import TaskDef, StageDef, InterfaceDef
from rasberry_coordination.coordinator_tools import logmsg, logmsgbreak, Rasberry_Logger


class TaskManager(object):

    def __init__(self, coordinator_ref):
        self.coordinator_ref = coordinator_ref
        self.toc_interface = InterfaceDef.TOC_Interface(coordinator_ref)

    def interrupt_task(self, agent_list):
        interrupts = {'pause': self.pause, 'resume': self.resume, 'reset': self.reset, 'disconnect': self.disconnect}
        logmsg(category="null")
        logmsg(category="DTM", msg="Interrupt detected!", speech=False)
        [logmsg(category="DTM", msg="    | %s : %s" % (a.agent_id, a.interruption[0])) for a in agent_list.values() if a.interruption]
        interrupt_ids = [a.agent_id for a in agent_list.values() if a.interruption and a.interruption[0] in interrupts]
        for aid in interrupt_ids: interrupts[agent_list[aid].interruption[0]](agent_list[aid], agent_list)


    def pause(self, agent, agent_list):
        """ Agent pausing works as follows:
        1. Put the active stage into a suspended state (so once active again it will be restarted)
        2. Add an additional pause stage which queries agent registration
        """
        agent().suspend()  # suspend active stage
        if agent().get_class() != "base.Pause":
            agent['stage_list'].insert(0, StageDef.Pause(agent))  # add paused stage

        scope = agent.interruption[3]
        agent().pause_state[scope[0].lower()] = True
        logmsg(category="DTM", msg="      | pause trigger ['%s'] set to True" % scope)
        logmsg(category="DTM", msg="      | stage state: %s" % agent().__repr__())

        agent.interruption = None  # reset interruption trigger
        agent.format_marker(color='red')


    def resume(self, agent, agent_list):
        """ Agent unpausing works as follows:
        1. Set the flag to end the pause stage (self.agent.registration)
        """
        logmsg(category="DTM", id=agent.agent_id, msg="Task advancement resumed.")

        scope = agent.interruption[3]
        # agent.registration = True  # enable generic query success condition
        if agent().get_class() == "base.Pause":
            agent().pause_state[scope[0].lower()] = False
            logmsg(category="DTM", msg="      | pause trigger ['%s'] set to False" % scope)
            logmsg(category="DTM", msg="      | stage state: %s" % agent().__repr__())

        agent.interruption = None  # reset interruption trigger


    def reset(self, agent, agent_list):
        """ Reset task works as follows:
        If the reset request comes from the initiator:
        - initiator deletes task
        - responder deletes task
        If the reset request comes from the responder:
        - initiator restarts task
        - responder deletes task
        """
        logmsg(category="DTM", msg="Request made to reset task: %s" % agent['id'])
        logmsg(category="DTM", msg="    | Task Details: %s" % agent.task)
        init, resp, tid = agent['initiator_id'], agent['responder_id'], agent['id']

        logmsg(category="DTM", msg="If the request came from TOC, it needs to release both?")

        if agent.agent_id == init:
            TaskDef.release_task(agent_list[init])
            self.unregister(init, agent_list)
        elif agent.agent_id == resp:
            TaskDef.restart_task(agent_list[init])
            self.unregister(resp, agent_list)
        agent_list[init].interruption = None  # reset interruption trigger

        if resp and resp in agent_list.keys():
            logmsg(category="DTM", msg="    | responder exists: %s" % (resp))
            TaskDef.release_task(agent_list[resp])
            agent_list[resp].interruption = None  # reset interruption trigger
        else:
            logmsg(category="DTM", msg="    | responder does not exist: %s" % (resp))
            print()

        self.toc_interface.EndTask([tid])


    def unregister(self, agent_id, agent_list):
        logmsg(category="DTM", msg="    | unregistering agent: %s" % agent_id)
        agent_list[agent_id].registration = False
        agent_list[agent_id].format_marker(color='red')


    def disconnect(self, agent, agent_list):
        a = weakref.ref(agent);
        del agent
        logmsg(level="error", category="DRM", id=a().agent_id, msg="Agent has been removed from coordinator.")
        logmsg(level="error", category="DRM", msg="    - Coordinator is no longer recieving location data")
        logmsg(level="error", category="DRM", msg="    - Agent is no longer reserving a node in the network")
        logmsg(level="error", category="DRM", msg="    - Ensure agent is moved out of the topology")
        a().delete_known_references(self.coordinator_ref)
