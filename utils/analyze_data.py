import numpy as np

d = np.load('data/default_walk_arm.npz', allow_pickle=True)
q = d['qpos']
names = d['joint_names']

print('Shape:', q.shape)
print('\nRange per joint:')
for i, n in enumerate(names):
    print(f'  {n}: min={q[:,i].min():.3f}, max={q[:,i].max():.3f}')

print('\nMax diff between consecutive frames:')
diff = np.abs(np.diff(q, axis=0))
for i, n in enumerate(names):
    max_jump = diff[:,i].max()
    max_jump_frame = diff[:,i].argmax()
    print(f'  {n}: max_jump={max_jump:.3f} at frame {max_jump_frame}')

# Find frames with sudden large jumps (>0.5 rad)
print('\nFrames with jumps > 0.3 rad:')
for i, n in enumerate(names[:4]):  # Only check left arm
    jumps = np.where(diff[:,i] > 0.3)[0]
    if len(jumps) > 0:
        print(f'  {n}: {len(jumps)} jumps at frames {jumps[:10]}...')
