
    """ Dynamic Task Management """
    @classmethod
    def release_task(cls, agent):
        logmsg(category="DTM", msg="    | releasing task %s" % (agent['name']))
        agent.task = None
    @classmethod
    def restart_task(cls, agent):
        logmsg(category="DTM", msg="    | restarting task %s" % (agent['name']))
        agent.add_task(task_name=agent['name'], task_id=agent['id'], index=0, quiet=True)
        agent.task = None


