#[derive(PartialEq)]
pub enum TestRecordingStatus {
    Ready = 1,
    WaitingToTakeTestRecording = 2,
    TakingTestRecording = 3,
    Recording = 4,
    TakingLongRecording = 5,
    WaitingToTakeLongRecording = 6,
}
