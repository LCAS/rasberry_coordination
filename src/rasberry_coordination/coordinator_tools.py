#! /usr/bin/env python
# ----------------------------------
# @author: jheselden
# @email: jheselden@lincoln.ac.uk
# @date:
# ----------------------------------

import rospy

def logmsgbreak(breaks=3):
    for i in range(breaks):
        logmsg(category="null")

def logmsg(level="info", category="OTHER", id="empty", msg='', throttle=0): #msg_color=default
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

    reject_tags = ["ROBNAV", "LIST1", "ROUTE", "ACTION1", "ROB_PY"]
    if category.upper() in reject_tags: return

    if use_custom_formatting:

        ros_time = ''
        if disable_ros_time_printout:
            ros_time = '\b' * 21

        """ Define id and/or category to highlight """
        color_id = ["storage01", "thorvald_001"]
        color_category = ["DTM", "DRM", "TOC"]  # TODO move these out of this definition and into some config file  # TODO: moving them outside the funciton will set on import logmsg? if so, we can grab from param server?
        # (load from parameter server in launch file?)

        """ Format category portion of message """
        valid_categories = ["ROBOT", "PICKER", "TASK", "OTHER", "ROB_PY", "ROUTE", "ACTION", "LOG", "STAGE", "SETUP", "RVIZ", "ROBNAV", "DRM", "DTM", "TOC"]
        total_pad_space = max([len(_category) + 1 for _category in valid_categories])
        if category.upper() in valid_categories:
            category_padding = total_pad_space - len(category)
            level_padding = (len(level) - 4)
            cat = category.upper() + (" " * (category_padding - level_padding))
        elif category.upper() == "NULL":
            cat = " " * total_pad_space
        else:
            rospy.logerr("category "+category.upper()+" not registered")
            return

        """ Format ID with conditions for when category or id is empty """
        ids = " " * (13 - len(str(id))) + str(id) + ":"
        if id == "empty": ids = " " * 14

        """ Define color values for printing """
        reset = '\033[00m'
        info_color = '\033[38;5;231m'
        warn_color = '\033[38;5;136m'
        err_color = '\033[38;5;1m'
        green_highlight = '\033[01;32m'
        yellow_highlight = '\033[01;33m'

        # identify base color for message
        if level == "warn":
            base_color = warn_color
        elif level == "error":
            base_color = err_color
        else:
            base_color = info_color

        # define colors for output base
        c1 = reset + base_color
        c2 = reset + base_color
        c3 = reset + base_color
        c4 = reset + base_color

        # highlight id and/or category based on definitions
        if str(category).upper() in color_category:
            c1 = yellow_highlight
        if str(id) in color_id:
            c3 = green_highlight

        msg = ros_time + "%s%s%s|%s%s %s%s%s" % (c1, cat, c2, c3, ids, c4, msg, reset)
    else:
        if category == "null": return
        msg = category + " | " + str(id) + " | " + msg

    # log in different manners based on the severity level and throttling
    if throttle:
        throttles = {"info": rospy.loginfo_throttle,
                     "warn": rospy.logwarn_throttle,
                     "error": rospy.logerr_throttle}
        throttles[level](throttle, msg)
    else:
        logs = {"info": rospy.loginfo,
                "warn": rospy.logwarn,
                "error": rospy.logerr}
        logs[level](msg)


def remove(collection, item):
    """removes and returns a given item if it is found within a given collection"""
    if item in collection:
        dt = str(collection.__class__)
        if dt in ["<type 'list'>", "<type 'set'>"]:
            collection.remove(item)
            return item
        elif dt in ["<type 'dict'>"]:
            return collection.pop(item)


def add(collection, item):
    """add a given item if it is not already found within a given collection"""
    if item is None:
        return
    dt = str(collection.__class__)
    if dt in ["<type 'dict'>"]:
        collection[item[0]] = item[1]
    else:
        if item not in collection:
            if dt in ["<type 'list'>"]:
                collection.append(item)
            elif dt in ["<type 'set'>"]:
                collection.add(item)


def move(item, old, new):
    """move an item from one collection to another"""
    if str(new.__class__) == "<type 'dict'>":
        val = remove(collection=old, item=item)
        add(collection=new, item=[item, val])
    else:
        add(collection=new, item=remove(collection=old, item=item))



if __name__ == '__main__':
    d1 = {'a': 1, 'b': 2, 'c': 3}
    d2 = {'d': 4, 'e': 5}

    move(item='d', old=d2, new=d1)

    print(str(d1))
    print(str(d2))
