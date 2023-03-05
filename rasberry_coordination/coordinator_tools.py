#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os
import rospy

class Lock:
    def __init__(self): self.status = False
    def __enter__(self): self.status = True
    def __exit__(self, ty, va, tr): self.status = False


class Rasberry_Logger(object):

    def __init__(self, enable_task_logging):
        self.enable_task_logging = enable_task_logging
        self.AllAgentsList = None
        self.enable_task_logging = True
        self.task_progression_log = 'task_progression.csv' #logs to $HOME/.ros/task_progression.csv
        self.log_routes = True
        self.timestep = 0
        self.iteration = 0
        self.previous_log_iteration = ""
        self.current_log_iteration = ""

    """ Task Stage Logging """
    def log_linebreak(self):
        dash_lengths = [13, 13, 36, 20, 3]
        dashes = []
        dashes += [',|,'.join(['-'*dl for dl in dash_lengths[:3]])]
        dashes += [',|,'.join(['-'*dash_lengths[3] for a in range(len(self.AllAgentsList))])]
        dashes += ['-' *dash_lengths[4]]
        return dashes
    def log_init(self):
        with open(self.task_progression_log, 'w+') as log:
            log.write(' ,'*5+"|,Agents:\n")

        return ['Timestep','Iteration','Stage']+[a.agent_id for a in self.AllAgentsList.values()]
    def log_break(self):
        return ['' for a in range(3+len(self.AllAgentsList))]
    def log_iteration(self):
        return ['%s','%s']+['' for a in range(1 + len(self.AllAgentsList))]

    def log_value(self, detail):
        return ['','']+[a[detail] for a in self.AllAgentsList.values()]
    def log_not_none(self, detail):
        return ['', ''] + [a[detail] is not None for a in self.AllAgentsList.values()]

    def log_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a['stage_list']:
                stages += [a().get_class()]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_new_stage(self):
        stages = []
        for a in self.AllAgentsList.values():
            if a['stage_list']:
                stages += [a().new_stage]
            else:
                stages += [None]
        return ['', ''] + stages
    def log_task(self):
        return ['', ''] + [a['name'] for a in self.AllAgentsList.values()]

    def log_summary(self, detail):
        lst=[]
        for a in self.AllAgentsList.values():
            switch = {'_start':a().new_stage,
                      '_notify_start':a().new_stage,
                      '_interaction':a().interaction_required,
                      '_query':a().stage_complete,
                      '_notify_end':a().stage_complete,
                      '_del':a().stage_complete}
            if switch[detail]:
                lst += [a().summary[detail]]
            else:
                lst += ['-']
        return ['','']+lst

    def log_data(self, switches, detail=None, linebreak=False):
        if not self.enable_task_logging:
            return

        switch_group_empty = {'init':self.log_init,
                              'break':self.log_break,
                              'iteration':self.log_iteration, #Make timestep query if time since last post has exceeded T
                              'task':self.log_task,
                              'stage': self.log_stage,
                              'new_stage':self.log_new_stage,
                              'linebreak': self.log_linebreak}
        switch_group_value = {'route':self.log_not_none,          #('route')
                              'interaction_required':self.log_value,   #('interaction_required')
                              'route_required':self.log_value,    #('route_required')
                              'stage_complete':self.log_value}    #('stage_complete')
        switch_group_summary = {'_start':self.log_summary,        #('_start')
                                '_notify_start':self.log_summary, #('_notify_start')
                                '_action':self.log_summary,       #('_action')
                                '_query':self.log_summary,        #('_query')
                                '_notify_end':self.log_summary,   #('_notify_end')
                                '_del':self.log_summary}          #('_del')


        for switch in switches:
            details = switch_group_empty[switch]() if switch in switch_group_empty else None
            details = switch_group_value[switch](switch) if switch in switch_group_value else details
            details = switch_group_summary[switch](switch) if switch in switch_group_summary else details

            for idx, item in enumerate(details[2:]):
                if item in [False, None]:
                    details[idx+2] = '-'

            if switch in ["break", "iteration"]:
                details.insert(2,'')
            elif switch in ["init", "linebreak"]:
                pass
            else:
                details.insert(2, switch)
                details += ['']
            details = [str(d) for d in details]
            self.current_log_iteration += "%s\n" % ',|,'.join(details)
    def publish_log(self):
        #TODO: add extra flag to set is ANY log returns a value? or query against empty log?
        if self.enable_task_logging:
            if self.previous_log_iteration != self.current_log_iteration:
                with open(self.task_progression_log, 'a') as log:
                    log.write(self.current_log_iteration % (self.timestep, self.iteration)) #use rospy.Time.now() ?
                    self.iteration += 1
                    logmsgbreak()
                    # logmsg(category="log", msg="Updating log")
                    print('\033[07m------------------------------------------\033[00m')

            self.previous_log_iteration = self.current_log_iteration
            self.current_log_iteration = ""

    def log_minimal(self, idx, AAL):
        self.AllAgentsList = AAL
        switch = {-2: self.publish_log,
                  -1:['init'],
                  0: ['linebreak', 'iteration', 'task', 'stage', 'new_stage', 'break'],
                  1: ['_start', '_notify_start', 'break'],
                  2: ['_action', 'break'],
                  3: ['route_required'],
                  4: ['route', 'break'],
                  5: ['_query', 'break', '_notify_end', '_del']}

        if idx == min(switch):
            switch[idx]()
        else:
            self.log_data(switch[idx])

        self.AllAgentsList = None
