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
    def transportation_request(cls, agent, details):
        agent.task_details = details
        agent.task_stage_list = [
            ("start_task",     StageDef.StartTask()),
            ("assign_courier", StageDef.AssignCourier(agent)),
            ("await_courier",  StageDef.AwaitCourier(agent)),
            ("load_courier",   StageDef.LoadCourier(agent))
        ]
    @classmethod
    def transportation_courier(cls, agent, details):
        agent.task_stage_list = [
            ("start_task",      StageDef.StartTask()),
            ("transit_picker",  StageDef.Navigation(agent)),
            ("loading",         StageDef.AwaitLoad(agent)),
            ("assign_storage",  StageDef.AssignStorage(agent)),
            ("storage_access",  StageDef.AwaitStoreAccess(agent)),
            ("transit_storage", StageDef.Navigation(agent)),
            ("unloading",       StageDef.AwaitUnload(agent))
        ]
    @classmethod
    def transportation_storage(cls, agent, details):
        agent.task_stage_list.append(
            ("start_task",            StageDef.StartTask()),
            ("await_courier_arrival", StageDef.AwaitCourierEntry(agent)),
            ("unload_courier",        StageDef.UnloadCourier(agent)),
            ("await_courier_exit",    StageDef.AwaitCourierExit(agent))
        )

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
