# gcode_interpreter/msg/Param.msg
# G-code의 파라미터 하나를 표현 (예: X100.0)

string key    # 파라미터 문자 (예: "G", "X", "Y", "F")
float64 value # 파라미터 값   (예: 1.0, 100.0, 50.0, 1000.0)