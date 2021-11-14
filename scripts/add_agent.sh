#!/bin/bash
high=12
rows=$(stty size | cut -d' ' -f1)
[ -z "$rows" ] && rows=$high
[ $rows -gt $high ] && rows=$high
cols=$(stty size | cut -d' ' -f2)

#rosmsg info rasberry_coordination/AgentDetails
#string agent_id
#rasberry_coordination/AgentSetup setup
#  rasberry_coordination/Module[] modules
#    string name
#    string role
#  bool has_presence
#  rasberry_coordination/KeyValuePair[] properties
#    string key
#    string value

export WHIPTAIL_TITLE="Add Agent to Coordinator"
export MODULES=$(echo NAME ROLE)
export PROPERTIES=$(echo KEY VALUE)

function inp () { echo $(whiptail --title "$WHIPTAIL_TITLE" --inputbox "$1" $rows $((cols - 5)) 3>&1 1>&2 2>&3) ; }
function yesno () { echo $(whiptail --title "$WHIPTAIL_TITLE" --yesno "$1" $rows $((cols - 5)) 3>&1 1>&2 2>&3 ; echo $? ) ; }
function pubecho () { echo $(whiptail --title "$WHIPTAIL_TITLE" --yesno "What do you wish to do with the message?" --yes-button "Publish" --no-button "Echo" $rows $((cols - 5)) 3>&1 1>&2 2>&3 ; echo $? ) ; }
function module_menu () { ans=$(whiptail --title "$WHIPTAIL_TITLE" --menu "Current Modules:" --ok-button "Add a Module" --cancel-button "Continue" $rows $((cols - 5)) 5 3>&1 1>&2 2>&3 $MODULES) ; echo $ans ; }
function property_menu () { ans=$(whiptail --title "$WHIPTAIL_TITLE" --menu "Current Properties:" --ok-button "Add a Property" --cancel-button "Continue" $rows $((cols - 5)) 5 3>&1 1>&2 2>&3 $PROPERTIES) ; echo $ans ; }
function check_continue () {
  export CONTINUE=$(yesno "$1")
  if [ $CONTINUE -eq 1 ]; then
    exit
  fi
}

export SETUP="rostopic pub /rasberry_coordination/dfm/add_agent rasberry_coordination/AgentDetails"
echo $SETUP


export AGENT_NAME=$(inp "Enter robot name")
export WHIPTAIL_TITLE="Add Agent ($AGENT_NAME) to Coordinator"
export AGENT="'agent_id': '$AGENT_NAME'"
check_continue "Add agent: $AGENT_NAME"
echo $AGENT


export PRESENCE_VALUE=$(yesno "Does agent ($AGENT_NAME) have physical presence?")
export PRESENCE="'has_presence': $PRESENCE_VALUE"
echo $PRESENCE


export MODULE="'modules': ["
while [ "" != "$(module_menu)" ]
do
  export NAME=$(inp "Enter module name:")
  export ROLE=$(inp "Enter role in $NAME")
  if [ 0 -eq $(yesno "Add module details: [ $NAME | $ROLE ]") ]; then
    export MODULES=$MODULES\ $(echo $NAME $ROLE )
    export MODULE=$MODULE$(echo "{'name': '$NAME', 'role': '$ROLE'}, ")
  fi
done
export MODULE=$MODULE$(echo "]")
echo $MODULE


export PROPERTY="'properties': ["
while [ "" != "$(property_menu)" ]
do
  export KEY=$(inp "Enter property key:")
  export VALUE=$(inp "Enter value for $KEY")
  if [ 0 -eq $(yesno "Add property: [ $KEY | $VALUE ]") ]; then
    export PROPERTIES=$PROPERTIES\ $(echo $KEY $VALUE )
    export PROPERTY=$PROPERTY$(echo "{'key': '$KEY', 'value': '$VALUE'}, ")
  fi
done
export PROPERTY=$PROPERTY$(echo "]")
echo $PROPERTY


export MESSAGE="{$AGENT, 'setup': {$MODULE, $PRESENCE, $PROPERTY}}"
if [ $(pubecho) -eq 1 ]; then
  echo "\n\nRostopic Command to Load Agent into Coordinator\n"
  echo $SETUP \"$MESSAGE\" -1
  exit
fi
$SETUP $MESSAGE -1





















