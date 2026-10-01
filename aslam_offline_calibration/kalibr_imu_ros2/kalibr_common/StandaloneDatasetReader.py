"""Lazy offline dataset adapters for the original camera/IMU optimizer."""
import csv
import json
from pathlib import Path
import numpy as np
import aslam_cv as acv
import _kalibr_image


class DatasetIterator:
    """Iterate over selected observations without retaining image payloads."""

    def __init__(self, dataset, indices):
        """Bind an independent cursor to the selected observation indices."""
        self.dataset, self.indices = dataset, iter(indices)

    def __iter__(self):
        """Return this cursor."""
        return self

    def __next__(self):
        """Read one selected observation."""
        return self.dataset.read(next(self.indices))


class Dataset:
    """Common compact index and timestamp filtering for cameras and IMU."""

    def __init__(self, bagfile, topic, bag_from_to=None, perform_synchronization=False, bag_freq=None):
        """Load one stream index and apply the original header-time filters."""
        if perform_synchronization:
            raise ValueError("Offline calibration preserves sensor header timestamps")
        self.bagfile, self.topic = bagfile, topic
        self.root = Path(bagfile)
        metadata = json.loads((self.root / "dataset.json").read_text())
        entry = metadata["streams"][topic]
        self.index_path = self.root / entry["index"]
        with self.index_path.open(newline="") as handle:
            self.rows = list(csv.reader(line for line in handle if not line.startswith("#")))
        self.stamps = np.array([int(row[0]) for row in self.rows], dtype=np.int64)
        if len(self.stamps) == 0 or np.any(np.diff(self.stamps) <= 0):
            raise ValueError("Missing or non-monotonic sensor timestamps")
        self.indices = list(range(len(self.rows)))
        if bag_from_to:
            if not 0 <= bag_from_to[0] < bag_from_to[1]:
                raise ValueError("Invalid time interval")
            first = int(self.stamps[0])
            self.indices = [i for i in self.indices if bag_from_to[0] <= (int(self.stamps[i]) - first) / 1e9 <= bag_from_to[1]]
        if bag_freq:
            if not np.isfinite(bag_freq) or bag_freq <= 0:
                raise ValueError("Invalid extraction frequency")
            selected, last = [], None
            for i in self.indices:
                stamp = int(self.stamps[i])
                if last is None or stamp - last >= 1e9 / bag_freq:
                    selected.append(i)
                    last = stamp
            self.indices = selected
        self.index = self.indices

    def time(self, index):
        """Construct an ASLAM timestamp from integer nanoseconds."""
        stamp = int(self.stamps[index])
        return acv.Time(stamp // 1000000000, stamp % 1000000000)

    def numImages(self):
        """Return the selected observation count."""
        return len(self.indices)

    def __iter__(self):
        """Create a fresh lazy observation iterator."""
        return self.readDataset()

    def readDataset(self):
        """Return a sequential iterator over the selected observations."""
        return DatasetIterator(self, self.indices)

    def readDatasetShuffle(self):
        """Return a shuffled iterator without changing the stored stream index."""
        indices = self.indices.copy()
        np.random.shuffle(indices)
        return DatasetIterator(self, indices)


class BagImageDatasetReader(Dataset):
    """Load lossless cached pixels for one camera only when requested."""

    def read(self, index):
        """Return the original timestamp and grayscale frame."""
        return self.time(index), _kalibr_image.read_gray(str(self.index_path.parent / self.rows[index][1]))

    def getImage(self, index):
        """Provide the original Kalibr dataset method."""
        return self.read(index)


class BagImuDatasetReader(Dataset):
    """Read compact SI-unit angular velocity and acceleration observations."""

    def read(self, index):
        """Return timestamp, gyro in rad/s, and acceleration in m/s²."""
        values = np.array([float(value) for value in self.rows[index][1:]], dtype=np.float64)
        return self.time(index), values[:3], values[3:]

    def numMessages(self):
        """Return the selected IMU observation count."""
        return len(self.indices)

    def getMessage(self, index):
        """Provide the original Kalibr IMU dataset method."""
        return self.read(index)
