"""Continue an integration run after an explicitly reported stage failure."""
from py_trees.common import Status
from py_trees.decorators import Decorator


class ContinueAfterFailure(Decorator):
    def update(self):
        result = self.decorated.status
        if result == Status.FAILURE:
            self.logger.warning('INTEGRATION_FORCED_CONTINUE stage=%s result=FAILURE' % self.decorated.name)
            return Status.SUCCESS
        return result
