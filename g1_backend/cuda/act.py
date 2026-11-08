"""ACT device lifecycle; retries a chunk on CPU after a CUDA runtime failure."""

from ..acceleration import disable, enabled, torch_device


class ActDevice:
    def __init__(self, checkpoint):
        self.checkpoint = checkpoint
        self.device = torch_device()

    def preprocessor(self):
        from lerobot.processor import PolicyProcessorPipeline

        return PolicyProcessorPipeline.from_pretrained(
            str(self.checkpoint),
            config_filename='policy_preprocessor.json',
            overrides={'device_processor': {'device': self.device}},
            local_files_only=True,
        )

    def use_cpu(self, owner):
        self.device = 'cpu'
        owner.device = 'cpu'
        owner.policy.to('cpu')
        owner.pre = self.preprocessor()

    def chunk(self, owner, state, image):
        if self.device == 'cuda' and not enabled():
            self.use_cpu(owner)
        try:
            return owner._chunk(state, image)
        except (RuntimeError, MemoryError) as error:
            if self.device != 'cuda':
                raise
            disable('torch', error)
            self.use_cpu(owner)
            return owner._chunk(state, image)
