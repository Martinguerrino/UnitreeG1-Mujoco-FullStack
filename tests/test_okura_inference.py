"""Optional integration tests: skipped unless dependencies and weights are installed."""

import numpy as np
import pytest

from experiments.okura.checkpoint import DEFAULT_CHECKPOINT


@pytest.fixture(scope='module')
def policy():
    pytest.importorskip('lerobot')
    pytest.importorskip('torch')
    if not (DEFAULT_CHECKPOINT / 'model.safetensors').exists():
        pytest.skip('Run bash scripts/setup_okura.sh to download optional ACT weights')
    from experiments.okura.infer import OkuraPolicy

    return OkuraPolicy(DEFAULT_CHECKPOINT)


def test_original_weights_produce_finite_repeatable_actions(policy):
    from safetensors.numpy import load_file

    stats = load_file(
        str(DEFAULT_CHECKPOINT / 'policy_preprocessor_step_3_normalizer_processor.safetensors')
    )
    state = stats['observation.state.mean'].copy()
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    first = policy.chunk(state, image)
    second = policy.chunk(state, image)
    assert first.shape == (100, 16)
    assert np.isfinite(first).all()
    np.testing.assert_allclose(first, second, atol=1e-6)


def test_reject_wrong_state_shape(policy):
    with pytest.raises(ValueError, match='16 finite'):
        policy.chunk(np.zeros(8), np.zeros((480, 640, 3), dtype=np.uint8))


def test_reject_wrong_image_dtype(policy):
    with pytest.raises(ValueError, match='RGB uint8'):
        policy.chunk(np.zeros(16), np.zeros((480, 640, 3), dtype=np.float32))
