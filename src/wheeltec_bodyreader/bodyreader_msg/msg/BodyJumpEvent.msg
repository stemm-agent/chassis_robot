uint8 PHASE_PENDING=1
uint8 PHASE_REBASED=2
uint8 PHASE_RECOVERING=3
uint8 PHASE_RELEASED=4
uint8 PHASE_EXPIRED=5
uint8 PHASE_RESET=6

uint8 OUTCOME_NONE=0
uint8 OUTCOME_REBASED=1
uint8 OUTCOME_RETURNED_OLD=2
uint8 OUTCOME_TIMED_OUT=3
uint8 OUTCOME_SOURCE_RESET=4

uint64 source_epoch
uint64 event_id
uint32 update_seq
int32 native_body_id
int32 origin_persistent_body_id
int32 current_persistent_body_id
float32 trusted_depth_mm
float32 observed_depth_mm
float32 confirmed_depth_mm
uint8 phase
uint8 outcome
uint8 good_count
