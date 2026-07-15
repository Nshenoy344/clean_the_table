import threading


def send_goal_and_wait(action_client, goal, timeout=None):
    """
    Send an action goal and block the calling thread until a result is
    available.

    Returns (accepted, result). `result` is None if the goal was
    rejected or if `timeout` elapsed before completion.
    """
    done_event = threading.Event()
    outcome = {'accepted': False, 'result': None}

    def _on_result(future):
        outcome['result'] = future.result()
        done_event.set()

    def _on_goal_response(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            done_event.set()
            return
        outcome['accepted'] = True
        goal_handle.get_result_async().add_done_callback(_on_result)

    action_client.send_goal_async(goal).add_done_callback(_on_goal_response)

    if not done_event.wait(timeout):
        return False, None
    return outcome['accepted'], outcome['result']


def call_service_and_wait(service_client, request, timeout=None):
    """
    Call a service and block the calling thread until the response is
    available. Returns None if the call fails or `timeout` elapses.
    """
    done_event = threading.Event()
    outcome = {'response': None}

    def _on_response(future):
        outcome['response'] = future.result()
        done_event.set()

    service_client.call_async(request).add_done_callback(_on_response)

    if not done_event.wait(timeout):
        return None
    return outcome['response']