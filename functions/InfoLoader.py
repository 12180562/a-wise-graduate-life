#!/usr/bin/env python
# -*- coding: utf-8 -*-

class InfoLoader:
    """
    It generates the ship data/information loader instance

    Attributes:
        num_ships:
            - The number of the ships to be simulated
            - Data type: int
        shipName_all: 
            - The list of the ship names to be simulated.
            - Data type: list
            - Example: [OS, TS1, TS2]

    Private methods:
        __get_num_ships(self., shipInfo_all): It returns the number of the ships to be simulated
        __get_ship_names(self, num_ships): It returns the ships names to be simulated
    """
    def __init__(self, shipInfo_all): 
        """
        It initialize the ship information loader.

        Args: 
            shipInfo_all: The condition of all the ships defined in `/params/main_parameter.yaml`
        """
        self.num_ships = int(self.__get_num_ships(shipInfo_all))
        self.shipName_all = self.__get_ship_names(self.num_ships)

    def __get_num_ships(self, shipInfo_all):
        """
        It returns the number of the ships to be simulated.

        Args: 
            shipInfo_all: The condition of all the ships defined in `/params/main_parameter.yaml`

        Returns:
            len(shipInfo_all): The number of the ships defined in `/params/main_parameter.yaml`
        """
        return len(shipInfo_all)

    def __get_ship_names(self, num_ships):
        """
        It returns the name of the ships for the simulation.

        Args:
            num_ships: The number of the ships for the simulation.

        Returns:
            shipName_all: 
                - The name of the ships for the simulation.
                - Data type: list
                - Example: ['OS', 'TS1', 'TS2']
        """
        shipName_all = [0 for _ in range(num_ships)]
        for idx in range(num_ships):
            # if idx == 0:
            #     shipName = "OS"
            # else:
            shipName = "ship" + str(idx + 1)
            shipName_all[idx] = shipName

        return shipName_all
