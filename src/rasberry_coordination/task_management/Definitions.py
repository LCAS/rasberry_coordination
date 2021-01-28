class StageDef(object):  #TODO: def StageDef.print_inheritence(): + populate stage_name.__delete__()
    class StageBase(object):
        def __repr__(self):
            return str(self.__class__).replace("<class '__main__.","").replace("'>","")
        def __init__(self, agent):
            self.agent = agent
        def query(self):
            success_conditions = []# What should be queried to tell if the stage is completed?
            self.agent.flag = any(success_conditions)
        def set_nav_target(self):
            self.agent.target = self.agent.current_node
            #self.agent.target = self.agent.location
        def __del__(self):
            # remove goal node?
            # remove routes?
            # remove? ?
            # communication with agent?
            pass

    """ Meta Stages """
    class Pause(StageBase):
        def query(self):
            success_conditions = [self.agent.registration]
            self.agent.flag(any(success_conditions))

    """ Move to Target Stages """
    class TargetedMovement(StageBase):
        def __repr__(self):
            cls_name = str(self.__class__).replace("<class '__main__.", "").replace("'>", "")
            return "%s(%s)"%(cls_name,self.target)
        def __init__(self, agent, target):
            super(StageDef.TargetedMovement, self).__init__(agent)
            self.target = target
        def set_nav_target(self):
            self.agent.target = self.target
        def __del__(self):
            self.agent.target = None
    class Navigation(TargetedMovement):
        def query(self):
            success_conditions = [self.agent.current_node == self.target]
            self.agent.flag(any(success_conditions))
    class EdgeUVTreatment(TargetedMovement):
        def query(self):
            success_conditions = [self.agent.current_node == self.target]
            self.agent.flag(any(success_conditions))
    class RowUVTreatment(TargetedMovement):
        def query(self):
            success_conditions = [self.agent.current_node == self.target]
            self.agent.flag(any(success_conditions))
    class EdgeDataCollection(TargetedMovement):
        def query(self):
            success_conditions = [self.agent.current_node == self.target]
            self.agent.flag(any(success_conditions))
    class RowDataCollection(TargetedMovement):
        def query(self):
            success_conditions = [self.agent.current_node == self.target]
            self.agent.flag(any(success_conditions))

    """ Wait for Time Expiry Stages """
    class AwaitLoad(StageBase):
        def query(self):
            success_conditions = [self.agent.tray_loaded]
            self.agent.flag(any(success_conditions))
    class AwaitUnload(StageBase):
        def query(self):
            success_conditions = [not self.agent.tray_loaded]
            self.agent.flag(any(success_conditions))

    """  """
    class Wait(StageBase):
        def __init__(self, agent, timeout):
            super(StageDef.Wait, self).__init__(agent)
            self.wait_timeout = timeout
    class LoadCourier(Wait):
        def query(self):
            success_conditions = [Now() - self.agent.start_time > self.wait_timeout]
            self.agent.flag(any(success_conditions))
    class UnloadCourier(Wait):
        def query(self):
            success_conditions = [Now() - self.agent.start_time > self.wait_timeout]
            self.agent.flag(any(success_conditions))

    class AwaitCourier(StageBase):
        def query(self):
            success_conditions = [True]
            self.agent.flag(any(success_conditions))


class TaskDef(object):
    """ Definitions for Task Initialisation Criteria """
    # Consistent attributes accessible directly, task-specific
    # attributes accessed through details dict defined on task
    # stage start.
    # Task definitions are defined in the perspective of the
    # agent handling the task.
    @classmethod
    def transit_base_station(cls, agent, details):
        agent.task_stage_list = [
            ("transit_base_station", StageDef.Navigation(agent, details['base_station_node']))
        ]

    """ Logistics Transportation """
    @classmethod
    def transportation_courier(cls, agent, details):
        agent.task_stage_list = [
            ("transit_picker", StageDef.Navigation(agent, details['collection_location'])),
            ("loading", StageDef.AwaitLoad(agent, details['loading_timeout'])),
            ("transit_storage", StageDef.Navigation(agent, details['delivery_location'])),
            ("unloading", StageDef.AwaitUnload(agent, details['unloading_timeout']))
        ]
    @classmethod
    def transportation_request(cls, agent, details):
        agent.task_stage_list = [
            ("await_courier", StageDef.AwaitCourier(agent)),
            ("load_courier",  StageDef.LoadCourier(agent, details['delivery_duration']))
        ]
    @classmethod
    def transportation_storage(cls, agent, details):
        agent.task_stage_list = [
            ("await_courier",   StageDef.AwaitCourier(agent)),
            ("unload_courier",  StageDef.UnloadCourier(agent, details['delivery_duration']))
        ]

    """ UV Treatment """
    @classmethod
    def uv_edge_treatment(cls, agent, details):
        agent.task_stage_list = [
            ("navigation",    StageDef.Navigation(agent, details['edge_start_node'])),
            ("uv_treat_edge", StageDef.EdgeUVTreatment(agent, details['edge_end_node']))
        ]
    @classmethod
    def uv_row_treatment(cls, agent, details):
        agent.task_stage_list = [
            ("navigation",    StageDef.Navigation(agent, details['row_start_node'])),
            ("uv_treat_row",  StageDef.RowUVTreatment(agent, details['row_end_node']))
        ]

    """ Monitoring """
    @classmethod
    def data_collection_edge(cls, agent, details):
        agent.task_stage_list = [
            ("navigation",    StageDef.Navigation(agent, details['row_start_node'])),
            ("data_collection",  StageDef.EdgeDataCollection(agent, details['row_end_node']))
        ]
    @classmethod
    def data_collection_row(cls, agent, details):
        agent.task_stage_list = [
            ("navigation", StageDef.Navigation(agent, details['row_start_node'])),
            ("data_collection", StageDef.RowDataCollection(agent, details['row_end_node']))
        ]
