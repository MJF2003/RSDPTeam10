# %%
import sys
import collections
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

'''Run this with system python so it has access to the ros packages'''

def recursive_leaf_keys(ordered_dict):
    for key, value in ordered_dict.items():
        if isinstance(value, collections.OrderedDict):
            yield from recursive_leaf_keys(value)
        else:
            yield key

def headers_for_msg_type(
    msg_type_str,
    *,
    truncate_length=None,  # None ~= --full-length, 128 ~= default echo
    no_arr=False,          # True ~= --no-arr
    no_str=False           # (if you ever mimic a --no-str flag)
):
    msg_cls = get_message(msg_type_str)
    dummy_msg = msg_cls()
    od = message_to_ordereddict(
        dummy_msg,
        truncate_length=truncate_length,
        no_arr=no_arr,
        no_str=no_str,
    )
    return list(recursive_leaf_keys(od))

if __name__ == "__main__":
    msg_type_str = sys.argv[1]
    headers = headers_for_msg_type(msg_type_str)
    print(",".join(headers))


# %%



