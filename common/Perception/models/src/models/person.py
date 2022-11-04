class Person:
    """ A person class that saves all the information about a person that the robot meets """

    def __init__(self, name, fav_drink):
        self.name = name
        self.fav_drink = fav_drink


    def get_fav_drink(self):
        return self.fav_drink

    def __str__(self):
        return (
            f"name : {self.name}\n"
            f"fav drink : {self.fav_drink}\n"
        )