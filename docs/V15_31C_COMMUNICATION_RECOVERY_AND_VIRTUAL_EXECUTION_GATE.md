# V15.31C Communication Recovery and Virtual-First Execution Gate

## Required execution order

Every commanded joint target uses this fail-closed order:

1. Bind the request to the current hardware session, state instance, measured
   six-axis start pose, gravity authority and thermal state.
2. Build the immutable synchronized quintic trajectory recipe.
3. Run the virtual arm through the complete recipe and require the independent
   planned-path collision proof, joint-limit checks, timing checks and exact
   trajectory SHA-256 binding to pass.
4. Mint one `PLAN_TOKEN` for that exact start pose, target, trajectory and
   collision proof.
5. Recheck communication, temperature, gravity authority and start-pose drift.
6. Only then publish the synchronized real-arm trajectory.  Every worker checks
   the same token, trajectory hash, activation epoch and common start time.

Changing the measured pose, session identity, communication health, gravity
authority, temperature state, target or trajectory invalidates the token.  A
failed or recovered communication epoch therefore cannot resume an old target.

## GO-M8010 recovery state machine

The J1, J2 and J3/J4/J5 workers remain the owners of their physical buses.  A
sustained feedback loss now performs these steps inside the same process:

1. Close the domain command UDP socket immediately.
2. Fence the last activation epoch, discard the cached command and require a
   new preview.
3. Attempt terminal BRAKE, close the stale serial object and reopen the stable
   `/dev/serial/by-id` device with 100 ms to 2 s bounded exponential backoff.
4. Transmit BRAKE only.  Require five consecutive stationary, identity-correct,
   mode-correct, error-free, thermally safe frames.  J2 additionally requires
   the two motors to remain within the synchronization limit.
5. Clear only the transport fault.  Drive, envelope, thermal, synchronization
   and load-limit latches are never cleared by communication recovery.
6. Reopen the command socket.  A higher activation epoch backed by a new
   virtual preview is required before any FOC command can be accepted.

The worker reports recovery begin/reopen/ready events and terminal attempt and
success counters.

## DM-G6220 recovery state machine

The DM adapter is pinned by VID, PID and serial number.  On a failed SDK send or
module removal, the logger drops the stale SDK device, retries discovery using
100 ms to 2 s bounded backoff and never replays the interrupted frame.  The
first traffic on a newly opened adapter is three DISABLE frames followed by
refresh frames.

The J6 controller then fences the activation epoch, spends the active gravity
authority, discards the command and waits for five consecutive healthy
DISABLED frames.  It clears only the communication latch; all non-communication
faults remain latched.  A newly preflighted higher-epoch command is required for
subsequent execution.

## Acceptance expectations

- No automatic motion occurs after 24 V restoration or module reinsertion.
- Healthy communication may return automatically; motion authority does not.
- GO command ports disappear while the corresponding transport is recovering.
- A failed active DM send is not replayed on the replacement adapter.
- Old `PLAN_TOKEN`, gravity authority and activation epoch cannot resume.
- Recovery is per fault domain; a J6 adapter event does not restart J1/J2/J345.
- Terminal BRAKE/DISABLED evidence remains mandatory at controlled shutdown.

