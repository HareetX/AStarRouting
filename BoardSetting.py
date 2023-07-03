from dataclasses import dataclass, field

from typing import Dict

from DesignSetting import DesignSetting


@dataclass
class BoardSetting:
    """The ``project`` token defines a KiCad layout according to the project file format used in
        ``.kicad_pro`` files.

        """
    design_setting: str = ""
    """The ``design_setting`` token defines the design_setting"""

    def from_sexpr(self, exp: list):
        """Convert the given S-Expresstion into a Board object

                Args:
                    - exp (list): Part of parsed S-Expression ``(kicad_pcb ...)``

                Raises:
                    - Exception: When given parameter's type is not a list
                    - Exception: When the first item of the list is not kicad_pcb

                Returns:
                    - Board: Object of the class initialized with the given S-Expression
                """
        if not isinstance(exp, list):
            raise Exception("Expression does not have the correct type")

        # if exp[0] != 'kicad_pcb':
        #     raise Exception("Expression does not have the correct type")

        # exp_ = []
        # flag = False
        # items = []
        # for item in exp:
        #     if item == '[':
        #         flag = True
        #     elif ']' in item:
        #         flag = False
        #         exp_.append(items)
        #         items = []
        #     else:
        #         if item == ',':
        #             pass
        #         elif flag:
        #             items.append(item)
        #         else:
        #             exp_.append(item)

        object = self
        for i in range(len(exp)):
            item = exp[i]
            if item == '"design_settings":':
                object.design_setting = DesignSetting().from_sexpr(exp[i+1])

        return object