AddNodeTask.srv  QueryBattery.srv  QueryLocation.srv  README.md  StringList.srv  String.srv


#AddNodeTask.srv
Used by coordinator to accept UV and DC tasks directly.

#QueryXYZ.srv
Formatting for the following Query service files
Used by health_monitoring to send details to third party applications.

Structure:
- rasberry_coordination/ServiceRequest service
---
- rasberry_coordination/ServiceResponse response
- unique_response/type item


- #QueryBattery.srv
- #QueryLocation.srv


#StringList.srv


#String.srv
