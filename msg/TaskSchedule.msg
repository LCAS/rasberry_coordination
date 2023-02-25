# Date in which the series sould begin
int32 series_start
string series_start_hr
#note: date_to_start_recurring is also the first start time

# Period in which task is viable
int32 time_allowance
string time_allowance_hr
#e.g. weekends represented as duration of 2 days

# Total number of events in series
int32 total_events

# How long to wait inbetween event start_times
int32 frequency
string frequency_hr
