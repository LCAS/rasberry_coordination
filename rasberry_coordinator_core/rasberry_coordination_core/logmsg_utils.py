#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import os, yaml
from rasberry_coordination_core.coordinator_node import GlobalLogger

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
            print("category %s not registered"%category.upper())
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
        test_colour = '\033[4;7;1m'

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
            if str(category).upper() == "TEST":
                c4 = c4 + test_colour

        if str(id) in colour_id:
            c3 = colour_id[str(id)]

        basic_msg = msg
        msg = ros_time + "%s%s%s|%s%s%s %s%s%s" % (c1, cat, c2, c3, ids, reset, c4, msg, reset)
    else:
        if category == "null": return
        msg = category + " | " + str(id) + " | " + msg

    # log in different manners based on the severity level and throttling
    logs = {"debug": GlobalLogger.debug,
            "info":  GlobalLogger.info,
            "warn":  GlobalLogger.warn,
            "error": GlobalLogger.error,
            "fatal": GlobalLogger.fatal}
    logs[level](msg)


    if id=="empty": id='';
    # if speech: os.system('spd-say "%s, %s" -r 10 -t female2 -w'%(id, basic_msg));
    if speech: os.system('espeak "%s"'%basic_msg);




"""  LOAD CONFIGURATION FROM ENVVAR (WE COULD PROBABLY FORCE THIS AGAIN FROM A CORE COORDINATOR INTERFACECALL)  """
#import load_dats; interface.base.subscriber('/update_logmsg', lambda : load_data(new_filepath))


import subprocess


def is_rejected(data): #return true if d is rejected
    if 'reject' in data:
        return data['reject'] #True in file
    return False

def is_coloured(data):
    return ('colour' in data and data['colour'])

def colour(data):
    if data in ["true", "True", "TRUE", True]:
        return False
    return '\033[01;%s'%data


def load_data(file):
    config_file = file

    with open(config_file) as f:
        config_data = yaml.safe_load(f)

    global valid_categories
    global reject_categories
    global colour_categories
    global colour_id

    reject_categories = [D['name'] for D in config_data['categories'] if is_rejected(D)]
    valid_categories =  [D['name'] for D in config_data['categories'] if not is_rejected(D)]
    colour_categories =  {D['name']:colour(D['colour']) for D in config_data['categories'] if not is_rejected(D) and is_coloured(D)}

    colour_id = {D['name']:colour(D['colour']) for D in config_data['ids']}


load_data(file=os.getenv('LOGMSG_CONFIG', None))
