"""Select providers without coupling the simulator to policy implementations."""

import importlib


class PolicyUnavailable(RuntimeError):
    pass


def load_policy(config):
    provider = config.get('provider')
    if not provider:
        raise PolicyUnavailable(
            'Manipulation provider is not configured; ACT integration is deferred'
        )
    if provider == 'unitree_rl_lab':
        from .locomotion import UnitreeVelocityPolicy

        return UnitreeVelocityPolicy(config)
    if provider == 'holosoma_fastsac':
        from .holosoma import HolosomaFastSACPolicy

        return HolosomaFastSACPolicy(config)
    module, separator, name = provider.partition(':')
    if not separator:
        raise ValueError('Custom provider must be module:factory')
    # A provider is trusted local Python code, just like an experiment script.
    return getattr(importlib.import_module(module), name)(config)
