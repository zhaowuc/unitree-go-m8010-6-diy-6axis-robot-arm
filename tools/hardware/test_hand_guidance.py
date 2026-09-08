"""Small coupled inner-servo simulation, not a physical-arm qualification."""
import math

from hand_guidance import GuidanceProfile, HandGuidance


def test_coupled_hand_force_release_jitter_stale_and_no_windup():
    profile = GuidanceProfile()
    guide = HandGuidance(profile, (0.0,) * 6)
    q, dq = [0.0] * 6, [0.0] * 6
    def gravity(position):
        return [0.2 * math.cos(position[1]), 1.0 + 0.4 * math.sin(position[1])] + [0.0] * 4
    sensor_bias = [0.01, -0.02, 0, 0, 0, 0]
    frozen_bias = [-value for value in sensor_bias]
    history = [(0.0, q[:], dq[:], [g + b for g, b in zip(gravity(q), sensor_bias)])]
    now, step, previous = 0.0, 0, (0.0,) * 6
    final_refs, maximum_motion = [], [0.0] * 6
    while now < 14:
        dt = (0.008, 0.013, 0.009, 0.020)[step % 4]
        now += dt
        sample = next((item for item in reversed(history) if item[0] <= now - 0.020), history[0])
        stamp, measured_q, measured_dq, measured_tau = sample
        # The core never sees the hand-force variable below: only the frozen-
        # bias-corrected model-minus-delayed/quantized-motor-torque estimate.
        external_estimate = [g - motor - bias for g, motor, bias in zip(gravity(measured_q), measured_tau, frozen_bias)]
        out = guide.update(now, measured_q, measured_dq, external_estimate, source_time_s=stamp)
        assert out.fault is None, (now, out)
        assert max(abs(value) for value in out.dq_ref) <= math.radians(profile.speed_deg_s) + 1e-12
        assert all(abs(a-b) <= math.radians(profile.target_error_deg) + 1e-10 for a,b in zip(out.q_ref, measured_q))
        assert all(abs((a-b)/dt-v) < 1e-9 for a,b,v in zip(out.q_ref, previous, out.dq_ref))
        previous = out.q_ref
        hand = ([0.7, -1.0] + [0.0] * 4 if 1 <= now < 2.5 else
                [-0.6, 0.8] + [0.0] * 4 if 7 <= now < 8 else [0.0] * 6)
        g = gravity(q)
        motor = [max(-5.0, min(5.0, 80*(target-actual) + 18*(target_v-actual_v) + weight))
                 for target,actual,target_v,actual_v,weight in zip(out.q_ref,q,out.dq_ref,dq,g)]
        net = [m + h - weight - 0.8*v for m,h,weight,v in zip(motor,hand,g,dq)]
        # Positive-definite coupled inertia of J1/J2; remaining four axes are
        # independent inner position loops with the same gravity cancellation.
        determinant = 1.6 * 2.0 - 0.3**2
        ddq = [(2.0*net[0]-0.3*net[1])/determinant,
               (1.6*net[1]-0.3*net[0])/determinant] + net[2:]
        dq = [v + dt*a for v,a in zip(dq,ddq)]
        q = [value + dt*v for value,v in zip(q,dq)]
        maximum_motion = [max(a,abs(b)) for a,b in zip(maximum_motion,q)]
        measured = [round((value+bias)/0.005)*0.005 for value,bias in zip(motor,sensor_bias)]
        history.append((now, q[:], dq[:], measured))
        if now > 13:
            final_refs.append(out.q_ref)
        step += 1
    assert all(value > math.radians(0.5) for value in maximum_motion[:2])
    assert out.state == "holding" and out.dq_ref == (0.0,) * 6
    assert max(max(abs(a-b) for a,b in zip(ref,final_refs[0])) for ref in final_refs) < 1e-9

    saturated = HandGuidance(GuidanceProfile(speed_deg_s=30), (0.0,) * 6)
    now, previous_v = 0.0, (0.0,) * 6
    for step in range(800):
        dt = (0.007, 0.019)[step % 2]
        now += dt
        out = saturated.update(now, (0.0,)*6, (0.0,)*6, (100.0,)*6, source_time_s=now)
        assert out.fault is None
        assert all(abs(v-old) <= math.radians(10)*dt + 1e-9 for v,old in zip(out.dq_ref,previous_v))
        previous_v = out.dq_ref
    bound_goal = out.q_ref
    assert any(out.limited) and max(bound_goal) <= math.radians(2) + 1e-12
    for _ in range(80):
        now += 0.01
        out = saturated.update(now, (0.0,)*6, (0.0,)*6, (0.0,)*6, source_time_s=now)
    assert out.state == "holding" and max(abs(a-b) for a,b in zip(out.q_ref,bound_goal)) < 1e-8
    now += 0.01
    failed = saturated.update(now, (0.0,)*6, (0.0,)*6, (100.0,)*6, source_time_s=now-0.101)
    assert "STALE" in failed.fault and failed.q_ref == out.q_ref and failed.dq_ref == (0.0,)*6
    assert saturated.pause(now).fault == failed.fault  # A data-gap pause cannot clear a latched fault.
    assert saturated.update(now+0.01, (0.0,)*6, (0.0,)*6, (0.0,)*6, source_time_s=now+0.01).fault == failed.fault
    for now, stamp, torque in ((0.06, 0.06, (0.0,)*6), (0.01, 0.02, (0.0,)*6),
                               (0.01, 0.01, (0.0, 0.0, float("nan"), 0.0, 0.0, 0.0))):
        invalid_input = HandGuidance(profile, (0.01,)*6)
        fault = invalid_input.update(now, (0.01,)*6, (0.0,)*6, torque, source_time_s=stamp)
        assert fault.fault and fault.q_ref == (0.01,)*6 and fault.dq_ref == (0.0,)*6
    for invalid in (30.01, float("nan")):
        try:
            GuidanceProfile(speed_deg_s=invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted invalid speed profile")
