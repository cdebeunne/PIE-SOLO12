from abc import ABC, abstractmethod
import numpy as np

class ActuatorsTrajectory:
    def __init__(self, nbActuators=12):
        self.trajectoryElements = {"size": 0}
        self.nbActuators = nbActuators

        arrayType = type(np.array(0))
        self.entriesTypes = {"t":arrayType, "q":arrayType, "q_dot":arrayType, "torques":arrayType, "gains":arrayType}
        # Shapes of the entries. If the size if -1, it must be size. Indifferent if size is -2.
        self.entriesShapes = {"t":[1, -1], "q":[12, -1], "q_dot":[12, -1], "torques":[12, -1], "gains":[2, -1]}
    
    def getSize(self):
        return self.trajectoryElements['size']

    def printInfo(self):
        print("Trajectory Class:")
        for key, value in self.trajectoryElements.items():
            if type(value) is int or type(value) is float:
                print("\t{0}:\t{1}".format(key, value))
            else:
                print("\t{0}:\t{1}".format(key, type(value)))

    def addElement(self, entry, data):
        # Check if data type exists
        if entry in self.entriesTypes:
            if type(data) is not self.entriesTypes["entry"]:
                print("Wrong type for data of {0} ({1}, {2}).".format(entry, type(data), self.entriesTypes["entry"]))
                return False
        else:
            return False

        # Check for size compatibility
        if entry in self.entriesShapes:
            shapes = self.entriesShapes['entry']
            
            for i, s in enumerate(shapes):
                valid = True

                if s==-1 :
                    if self.trajectoryElements['size']<0:
                        self.trajectoryElements['size'] = data.shape[i]
                    else:
                        valid = self.trajectoryElements['size'] != data.shape[i]
                else:
                    valid = s != data.shape[i]

                if not valid:
                    print("Invalid size for entry {0} on axis {1} ({2}, {3})".format(entry, i, data.shape[i], s))
                    return False
        
        # Add the data to the trajectory
        self.trajectoryElements[entry] = data

        return True

    def getElement(self, entry):
        if not self.containsElement(entry):
            print("The entry {0} does not exist.".format(entry))
            

    def containsElement(self, entry):
        return entry in self.trajectoryElements

    def setDefaultElement(self, entry, data):
        pass

    def plotTrajectory(self):
        pass


class TrajectoryGenerator(ABC):
    def __init__(self):
        return

    @abstractmethod
    def generateTrajectory(self, **kwargs):
        """
        This function should return the trajectory generated using
        the kwargs parameters.
        Values returned must be the following a Trajectory class
        """
        pass