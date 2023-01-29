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


class RootInspector(object):

    def __init__(self, topic, root):
        from rasberry_coordination.srv import String as StringRequest
        self.root = root
        rospy.Service(topic, StringRequest, self.root_inspector_srv)

    def root_inspector_srv(self, req):
        from rasberry_coordination.srv import StringResponse as StringResponse
        resp = StringResponse()
        resp.success = True

        root = req.data
        d = root.split('.')[1:]
        tree = self.root
        for k in d:
            k = k.replace(']', '').split('[')

            if k[0] not in tree.__dict__: tree = tree.__dict__; break
            #Error processing request: 'dict' object has no attribute '__dict__'

            tree = tree.__getattribute__(k[0])
            if len(k) > 1:
                if type(tree) == type([]):
                    tree = tree[int(k[1])]
                elif type(tree) == type(dict()):
                    tree = tree[k[1]]
        resp.msg = str(tree)
        return resp


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


def logmsgbreak(total=3):
    """ Print a number of lines using logmsg formatting

    :param total: number of lines to print (default: 3)
    :return: None
    """

    for i in range(total):
        logmsg(category="null")


def logmsg(level="info", category="OTHER", id="empty", msg='', throttle=0, speech=False):
    """ Print formatted log messages to console.

    Functionality offered:
    > Clear division of category, id and message
    > Option to remove msg time
    > Option to highlight key details
    > Ability to filter out certain categories

    In order to retain speed and simplicity, properties are preset internally here for:
    :attr reject_categories: list of tags to ignore (useful for managing debugging tools)
    :attr colour_id: list of id's to colour
    :attr colour_categories: list of categories to colour
    :attr valid_categories: list of accepted categories (used to define column widths)
    :attr quick_print: boolean to define if logging should default to use print()
    :attr use_custom_formatting: boolean to define if logging should use basic rospy.logmsg or formatted variant
    :attr disable_ros_time_printout: boolean to define if logging should omit the rospy.Time.now() in logging (slower)
    :attr reset: text colour to return to after logmsg
    :attr info_colour: colour for rospy log info
    :attr warn_colour: colour for rospy log warn
    :attr err_colour: colour for rospy log error
    :attr green_highlight: highlight colour for colour_id
    :attr yellow_highlight: highlight colour for colour_categories

    :param level: level of verbosity [info, warn, error] [fatal and bug/debug not included]
    :param category: type of logging [robot, picker, task, note]
    :param id: agent_id or task_id used for identification
    :param msg: message to display
    :return: None
    """
    # if category == "DTM": speech=True

    quick_print = False
    if quick_print:
        if category == "null": print("\n")
        else: print(category + " | " + str(id) + " | " + msg)
        return

    use_custom_formatting = True
    disable_ros_time_printout = True  # Can cause visual issues on console such as below:
    # [INFO] [1605509085.140152]: OTHER  | var: 1	#output as false
    # [INFO] OTHER  | var: 1 						#ideal output if true
    # [INFO] OTHER  | var: 1152]:					#rostime char after end of ideal output appear (\b cant reach)
    # TODO: include padding at end of msg

    if category.upper() in reject_categories: return

    if use_custom_formatting:

        ros_time = ''
        if disable_ros_time_printout:
            ros_time = '\b' * 21  # TODO: swap out using \u001b[{n}D

        """ Format category portion of message """
        total_pad_space = max([len(_category) + 1 for _category in valid_categories])
        if category.upper() in valid_categories:
            category_padding = total_pad_space - len(category)
            level_padding = (len(level) - 4)
            cat = category.upper() + (" " * (category_padding - level_padding))
        elif category.upper() == "NULL":
            cat = " " * total_pad_space
        else:
            rospy.logerr("category %s not registered"%category.upper())
            return

        """ Format ID with conditions for when category or id is empty """
        total_pad_space = max([ len(_id) for _id in colour_id.keys() ]) + 1
        if id == "empty":
            ids = " " * (total_pad_space + 1) # +1 for :
        else:
            ids = " " * (total_pad_space - len(str(id))) + str(id) + ":"
            if id not in colour_id:
                colour_id[str(id)] = '\033[01;0m'


        """ Define colour values for printing """ #TODO: optimise this with re.sub(r'\[.*\]','[]',line)
        reset = '\033[00m'
        colour_template = '\033[01;%s'
        info_colour = '\033[38;5;231m\033[0m'
        warn_colour = '\033[38;5;136m'
        err_colour = '\033[38;5;1m'
        green_highlight = '\033[01;32m'
        yellow_highlight = '\033[01;33m'

        # identify base colour for message
        if level == "warn":
            base_colour = warn_colour
        elif level == "error":
            base_colour = err_colour
        else:
            base_colour = info_colour

        # define colours for output base
        c1 = reset + base_colour
        c2 = reset + base_colour
        c3 = reset + base_colour
        c4 = reset + base_colour

        # highlight id and/or category based on definitions
        if str(category).upper() in colour_categories:
            catcol = colour_categories[str(category).upper()]
            c1 = colour_template%catcol if catcol else yellow_highlight
        if str(id) in colour_id:
            c3 = colour_id[str(id)]

        basic_msg = msg
        msg = ros_time + "%s%s%s|%s%s%s %s%s%s" % (c1, cat, c2, c3, ids, reset, c4, msg, reset)
    else:
        if category == "null": return
        msg = category + " | " + str(id) + " | " + msg

    # log in different manners based on the severity level and throttling
    if throttle:
        throttles = {"debug": rospy.logdebug_throttle,
                     "info": rospy.loginfo_throttle,
                     "warn": rospy.logwarn_throttle,
                     "error": rospy.logerr_throttle,
                     "fatal": rospy.logfatal_throttle}
        throttles[level](throttle, msg)
    else:
        logs = {"debug": rospy.logdebug,
                "info": rospy.loginfo,
                "warn": rospy.logwarn,
                "error": rospy.logerr,
                "fatal": rospy.logfatal}
        logs[level](msg)


    if id=="empty": id='';
    # if speech: os.system('spd-say "%s, %s" -r 10 -t female2 -w'%(id, basic_msg));
    if speech: os.system('espeak "%s"'%basic_msg);


import subprocess
import rasberry_des.config_utils

config_file = os.getenv('LOGMSG_CONFIG', None)
if not config_file:
    result = subprocess.check_output('rospack find rasberry_coordination', shell=True)
    config_file = result[:-1]+"/src/rasberry_coordination/logging_config/logmsg.yaml"
config_data = rasberry_des.config_utils.get_config_data(config_file)

def is_rejected(data): #return true if d is rejected
    if 'reject' in data: return data['reject']
    else: return False
def is_not_rejected(data):  return not is_rejected(data)
def is_coloured(data):      return ('colour' in data and data['colour'])
def colour(data):
    if data in ["true", "True", "TRUE", True]: return False
    else: return '\033[01;%s'%data

valid_categories =  [D['name'] for D in config_data['categories'] if is_not_rejected(D)]
reject_categories = [D['name'] for D in config_data['categories'] if is_rejected(D)]
colour_categories =  {D['name']:colour(D['colour']) for D in config_data['categories'] if is_not_rejected(D) and is_coloured(D)}

colour_id = {D['name']:colour(D['colour']) for D in config_data['ids']}
