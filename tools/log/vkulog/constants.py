# SPDX-License-Identifier: GPL-3.0-or-later
"""Constants of the ULog file format version 1 (spec 0011, ADR-0019)."""

HEADER_BYTES = b"ULog\x01\x125"
FILE_VERSION = 1
SYNC_BYTES = b"/s\x13 %\x0c\xbb\x12"

MSG_TYPE_FORMAT = ord("F")              # 70
MSG_TYPE_DATA = ord("D")                # 68
MSG_TYPE_INFO = ord("I")                # 73
MSG_TYPE_INFO_MULTIPLE = ord("M")       # 77
MSG_TYPE_PARAMETER = ord("P")           # 80
MSG_TYPE_PARAMETER_DEFAULT = ord("Q")   # 81
MSG_TYPE_ADD_LOGGED_MSG = ord("A")      # 65
MSG_TYPE_REMOVE_LOGGED_MSG = ord("R")   # 82
MSG_TYPE_SYNC = ord("S")                # 83
MSG_TYPE_DROPOUT = ord("O")             # 79
MSG_TYPE_LOGGING = ord("L")             # 76
MSG_TYPE_LOGGING_TAGGED = ord("C")      # 67
MSG_TYPE_FLAG_BITS = ord("B")           # 66
