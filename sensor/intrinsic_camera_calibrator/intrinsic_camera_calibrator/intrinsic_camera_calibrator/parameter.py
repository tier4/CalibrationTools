import copy
import threading


class Parameter:
    def __init__(self, parameter_type, value, min_value, max_value):
        self._parameter_type = parameter_type
        self._value = parameter_type(value)
        self._min_value = parameter_type(min_value)
        self._max_value = parameter_type(max_value)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):

        new_value = self._parameter_type(value)

        if new_value < self.min_value or new_value > self.max_value:
            raise AttributeError("New value out of range")

        self._value = new_value

    @property
    def min_value(self):
        return self._min_value

    @property
    def max_value(self):
        return self._max_value

    @property
    def parameter_type(self):
        return self._parameter_type


class ParameteredClass:
    def __init__(self, lock: threading.RLock = threading.RLock(), cfg: dict = dict()):  # noqa C408

        self.lock = lock
        self.set_parameters(**cfg)

    def __deepcopy__(self, memodict={}):

        obj = type(self)(lock=self.lock)

        for k, v in vars(self).items():
            if k != "lock":
                setattr(obj, k, copy.deepcopy(v))

        return obj

    def set_parameters(self, **kwargs):
        with self.lock:
            for k, v in kwargs.items():
                if k in vars(self) and isinstance(vars(self)[k], Parameter):
                    old_val = getattr(self, k)
                    old_val.value = v

    def parameters(self) -> dict:

        with self.lock:
            p_dict = {}

            for k, v in vars(self).items():
                if isinstance(v, Parameter):
                    p_dict[k] = v

            return p_dict
