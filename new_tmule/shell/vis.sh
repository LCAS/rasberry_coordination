# RViZ Config
if ! $USE_SIM; then
  export_default RVIZ_CONFIG "server"
else
  export_default RVIZ_CONFIG "topological_navigation_server_multisim"
fi

# Site details
export_default METRIC_MAP "${SITE_PATH}/map.yaml"
export_default NOGO_MAP "${SITE_PATH}/nogo_map.yaml"

# Coordination configs
export_default TASK_COORD_CONFIG "${SITE_PATH}/coordination/rb_atm.yaml"



#  #  $ROBOT_NAME_1
#  #  $ROBOT_NAME_2

