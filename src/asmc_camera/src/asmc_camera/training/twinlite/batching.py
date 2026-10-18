"""Deterministic homogeneous-shape batches; every image is used once per epoch."""
from collections import Counter, defaultdict
import random


class ShapeBatchSampler:
    def __init__(self, rows, batch_size, seed=20261017, shuffle=True,
                 balance_conditions=False):
        if batch_size < 1:
            raise ValueError('batch_size must be positive')
        self.batch_size, self.seed, self.shuffle = int(batch_size), int(seed), bool(shuffle)
        self.epoch = 0
        self._cached_batches = None
        self.groups = defaultdict(list)
        self.conditions = defaultdict(list)
        self.balance_conditions = bool(balance_conditions)
        for index, row in enumerate(rows):
            self.groups[tuple(row['geometry']['target_hw'])].append(index)
            condition = (row.get('weather'), str(row.get('hour')))
            if condition[0] in ('SUNNY', 'FOGGY') and condition[1] in ('11', '13', '15'):
                self.conditions[condition].append(index)

    def _extra_counts(self):
        if not self.balance_conditions or not self.conditions:
            return {}
        target = max(map(len, self.conditions.values()))
        return {key: min(len(indices), target-len(indices))
                for key, indices in self.conditions.items()}

    def set_epoch(self, epoch):
        self.epoch = int(epoch)
        self._cached_batches = None

    def __len__(self):
        return len(self._batches())

    def _batches(self):
        if self._cached_batches is not None:
            return self._cached_batches
        rng = random.Random(self.seed + self.epoch)
        extra = Counter()
        for condition, count in sorted(self._extra_counts().items()):
            extra.update(rng.sample(self.conditions[condition], count))
        batches = []
        for shape, original in sorted(self.groups.items()):
            indices = [index for index in original for _ in range(1+extra[index])]
            if self.shuffle:
                rng.shuffle(indices)
            batches.extend(indices[i:i+self.batch_size] for i in range(0,len(indices),self.batch_size))
        if self.shuffle:
            rng.shuffle(batches)
        self._cached_batches = tuple(tuple(batch) for batch in batches)
        return self._cached_batches

    def __iter__(self):
        return iter(self._batches())
