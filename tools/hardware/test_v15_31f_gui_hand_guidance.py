from types import SimpleNamespace
import copy
import v15_31f_gui_hand_guidance as runtime


class Widget:
    def __init__(self, text=""):
        self.text, self.enabled, self.down = text, True, False
        self.pressed = self.released = self.clicked = SimpleNamespace(connect=lambda *_: None)
    def setEnabled(self, value): self.enabled = value
    def setWordWrap(self, value): pass
    def setText(self, value): self.text = value
    def isDown(self): return self.down


def test_release_freezes_each_native_target_before_ack_and_return_before_brake(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(runtime, "time", SimpleNamespace(monotonic_ns=lambda: int(clock[0]*1e9)))
    names = tuple(name for group in runtime.MOTOR_GROUPS for name in group)
    hardware = {"healthy": True, "source_monotonic_ns": int(clock[0]*1e9),
        "controller_mode_by_motor": dict.fromkeys(names, "hold"), "per_motor": {name: {} for name in names}}
    sent, actions = [], []
    sample = dict(healthy=True, feedback_fresh=True, authority=True, modes=hardware["controller_mode_by_motor"],
        stationary_hold_ready=True, router_hold_fresh=True, j6_drive_state=1, actual_rad=[0.01]*6)
    window = SimpleNamespace(node=SimpleNamespace(latest_hardware=hardware,
        latest_gravity_status={"empirical_validation": {"hand_guidance_authorized": True}}),
        teach_toolbar=SimpleNamespace(actions=lambda: [], addWidget=lambda *_: None),
        command_targets=[0.0]*6, targets=[0.0]*6, candidate_targets=[0.0]*6, activation_epoch=5,
        hardware_mode="hold", action_group_manual_buttons=[], machine=SimpleNamespace(hold=lambda: None),
        _set_virtual_editable=lambda *_: None, _action_group_health=lambda *_, **__: {},
        _action_group_hold_ready=lambda *_: True, isActiveWindow=lambda: True)
    window._authorize_active_joints = lambda *_: setattr(window, "activation_epoch", window.activation_epoch+1)
    window._publish_command = lambda **kwargs: sent.append((window.hardware_mode, window.activation_epoch,
        tuple(window.command_targets), copy.deepcopy(getattr(window, "hand_guidance_reference", None)), kwargs))
    window._emergency_brake = lambda **_: actions.append("brake")
    runner = SimpleNamespace(state="moving")
    def return_start(origin):
        actions.append("return")
        return SimpleNamespace(runner=runner)
    demo = runtime.GuidanceDemo(window, lambda: sample, lambda *_: None, lambda *_: None, lambda *_: None,
        gui=SimpleNamespace(QPushButton=Widget, QLabel=Widget), interactive_teach=True,
        start_center=return_start, now=lambda: clock[0])
    demo.interactive_ready, demo.origin, demo.bias, demo.guidance_phase = True, (0.0,)*6, (0.0,)*6, "ready"
    demo.drag_button.down = True
    demo.start_guidance()
    assert demo.guidance_phase == "engaging_guidance" and not sent
    epoch = demo.guide_epoch
    clock[0] += .02
    demo.tick()
    assert len(sent) == 1 and sent[-1][0] == "teach" and sent[-1][4] == {"guidance_owner": True}
    def feedback(targets, accepted_epoch, pause=None, mode="hold"):
        hardware["source_monotonic_ns"] = int(clock[0]*1e9)
        for index, group in enumerate(runtime.MOTOR_GROUPS):
            for name in group:
                hardware["controller_mode_by_motor"][name] = mode
                hardware["per_motor"][name] = {"accepted_guidance_metadata_status": "OBSERVED",
                    "accepted_guidance_target_rad": targets[index], "accepted_guidance_activation_epoch": accepted_epoch,
                    "feedback_source_monotonic_ns": int(clock[0]*1e9), "guidance_paused_reason": pause}
    # A partial admission must be cancelled while the original 500ms lease is
    # still alive. No moving reference has been sent, so the target is known.
    feedback([0.0]*6, epoch-1)
    for name in ("J1", "J6"):
        hardware["controller_mode_by_motor"][name] = "teach"
        hardware["per_motor"][name]["accepted_guidance_activation_epoch"] = epoch
    clock[0] += .14
    demo.tick()
    assert demo.guidance_phase == "hold_barrier" and sent[-1][0] == "hold"
    assert sent[-1][1] > epoch and sent[-1][2] == (0.0,)*6
    assert sent[-1][3]["velocity_rad_s"] == [0.0]*6 and not actions
    assert demo.guidance_events[-1]["reason"] == "ALL_DOMAIN_ACK_TIMEOUT"
    clock[0] += .02
    feedback([0.0]*6, window.activation_epoch)
    demo.tick()
    assert demo.guidance_phase == "ready"
    # A quick release also cancels immediately instead of waiting forever for
    # an entry ACK that a rejecting domain will never send.
    demo.start_guidance()
    quick_epoch = demo.guide_epoch
    demo.release_guidance()
    assert demo.guidance_phase == "hold_barrier" and window.activation_epoch > quick_epoch
    clock[0] += .02
    feedback([0.0]*6, window.activation_epoch)
    demo.tick()
    assert demo.guidance_phase == "ready" and not actions
    demo.start_guidance()
    epoch = demo.guide_epoch
    clock[0] += .02
    feedback([0.0]*6, epoch, mode="teach")
    demo.tick()
    assert demo.guidance_phase == "guiding"
    # The observed 54.9568ms Qt delay holds q/v0 and resumes on a fresh tick,
    # without relaxing the core's 50ms integration bound or clearing faults.
    def observation():
        return SimpleNamespace(source_monotonic_ns=int(clock[0]*1e9), q=(0.0,)*6,
            dq=(0.0,)*6, residual_nm=(0.0,)*6)
    monkeypatch.setattr(runtime, "matched_observation", lambda *_, **__: observation())
    clock[0] += .0549568
    demo.tick()
    assert demo.guidance_phase == "guiding" and demo.guidance_fault is None
    assert demo.guidance_events[-1]["reason"] == "CONTROL_TICK_DELAY"
    assert sent[-1][2] == (0.0,)*6 and sent[-1][3]["velocity_rad_s"] == [0.0]*6
    clock[0] += .02
    demo.tick()
    clock[0] += .02
    demo.tick()
    assert demo.input_wait_since is None and demo.guidance_fault is None
    # A missing measurement must never renew yesterday's moving reference.
    window.hand_guidance_reference = demo.reference([0.1]*6)
    monkeypatch.setattr(runtime, "matched_observation", lambda *_, **__: None)
    clock[0] += .02
    demo.tick()
    assert demo.guidance_phase == "guiding" and demo.input_wait_since == clock[0]
    assert sent[-1][2] == (0.0,)*6
    assert sent[-1][3]["velocity_rad_s"] == [0.0]*6
    assert sent[-1][3]["freeze_reference"] is False
    clock[0] += .06  # Longer than the core integration dt, still within gap budget.
    observation = SimpleNamespace(source_monotonic_ns=int(clock[0]*1e9), q=(0.01,)*6,
        dq=(0.0,)*6, residual_nm=(0.0,)*6)
    monkeypatch.setattr(runtime, "matched_observation", lambda *_, **__: observation)
    demo.tick()
    assert demo.input_wait_since is None and window.command_targets == [0.0]*6
    clock[0] += .02
    demo.tick()
    assert demo.guidance_phase == "guiding" and demo.guidance_fault is None
    assert sent[-1][2] == (0.0,)*6  # Did not recapture displaced actual position.
    monkeypatch.setattr(runtime, "matched_observation", lambda *_, **__: None)
    clock[0] += .02
    demo.tick()
    clock[0] += .11
    demo.tick()
    assert demo.guidance_phase == "freezing" and sent[-1][3]["freeze_reference"] is True
    assert demo.guidance_fault.startswith("GUIDANCE_INPUT_GAP:")
    demo.guidance_fault = None
    demo.guidance_phase = "guiding"  # Continue the existing native freeze/ACK check.
    # Last published reference can differ from a domain's last accepted one.
    window.command_targets = [0.02]*6
    clock[0] += .02
    demo.drag_button.down = False
    # Qt can synchronously emit released when a held button is disabled.
    before_release = sum(e["event"] == "guidance_release" for e in demo.guidance_events)
    demo.drag_button.setEnabled = lambda enabled: (setattr(demo.drag_button, "enabled", enabled),
        demo.release_guidance() if not enabled else None)
    demo.release_guidance()
    assert sum(e["event"] == "guidance_release" for e in demo.guidance_events) == before_release + 1
    assert window.command_targets == [0.02]*6 and window.hand_guidance_reference["freeze_reference"] is True
    demo.tick()
    assert demo.guidance_phase == "freezing" and sent[-1][0] == "teach" and not actions
    clock[0] += .02
    targets = [0.009, 0.01, 0.011, 0.012, 0.01, 0.011]
    feedback(targets, epoch, pause="GUI_RELEASE")
    demo.tick()
    assert demo.guidance_phase == "hold_barrier" and sent[-1][0] == "hold"
    assert sent[-1][2] == tuple(targets) and window.activation_epoch > epoch
    clock[0] += .02
    demo.tick()  # Paused HOLD is insufficient; the newer ACK must be echoed.
    assert demo.guidance_phase == "hold_barrier"
    feedback(targets, window.activation_epoch, mode="hold")
    clock[0] += .02
    demo.tick()
    assert demo.guidance_phase == "ready" and not actions
    monkeypatch.setattr(runtime, "matched_observation", lambda *_, **__: SimpleNamespace(residual_nm=(0.0,)*6))
    assert demo.return_requested and not demo.ending and not actions
    clock[0] += .02
    demo.tick()
    clock[0] += .51
    demo.tick()
    assert actions == ["return"] and window.hand_guidance_reference is None
    runner.state = "complete"
    sample["stationary_hold_ready"] = False
    clock[0] += .02
    demo.tick()
    assert actions == ["return"]
    sample.update(stationary_hold_ready=True, actual_rad=[0.0]*6)
    clock[0] += .02
    demo.tick()
    assert demo.return_verified and actions == ["return"]
    assert demo.guidance_phase == "ready" and demo.terminal is None
    # A later soft warning stops new hand guiding but retains a healthy HOLD;
    # no force-model residual is used to invent a contact signal for return.
    empirical = window.node.latest_gravity_status["empirical_validation"]
    empirical.update(return_only=True, hand_guidance_authorized=False,
                     motion_warnings=["EMPIRICAL_J2_SYNC_WARNING"])
    clock[0] += .02
    demo.tick()
    clock[0] += .51
    demo.tick()
    assert demo.terminal is None and not demo.drag_button.enabled and actions == ["return"]
    empirical.update(return_only=False, hand_guidance_authorized=True)
    clock[0] += .02
    demo.tick()
    assert demo.drag_button.enabled
    window.command_targets = [0.0]*6
    demo.drag_button.down = True
    demo.start_guidance()
    assert demo.guidance_phase == "engaging_guidance"
    empirical.update(return_only=True, hand_guidance_authorized=False)
    clock[0] += .02
    demo.tick()
    assert demo.guidance_phase == "hold_barrier" and actions == ["return"]
    assert window.command_targets == [0.0]*6 and demo.return_requested
    # Hard authority loss still terminates even when a soft warning is present.
    sample["authority"] = False
    clock[0] += .02
    demo.tick()
    assert demo.failure == "hand guidance lost hardware/feedback/gravity authority"
    assert actions == ["return", "brake"]
