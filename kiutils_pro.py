from dataclasses import dataclass
from os import path
from typing import Optional

import re

from BoardSetting import BoardSetting
from NetSetting import NetSetting

dbg = False

term_regex = r'''(?mx)
    \s*(?:
        (?P<brackl>\{)|
        (?P<brackr>\})|
        (?P<num>[+-]?\d+\.\d+(?=[\ \)])|\-?\d+(?=[\ \)]))|
        (?P<sq>"(?:[^"]|(?<=\\)")*"(?:(?=\))|(?=\s)))|
        (?P<s>[^(^)\s]+)
       )'''


def parse_sexp(sexp):
    stack = []
    out = []
    if dbg: print("%-6s %-14s %-44s %-s" % tuple("term value out stack".split()))
    for termtypes in re.finditer(term_regex, sexp):
        term, value = [(t, v) for t, v in termtypes.groupdict().items() if v][0]
        if dbg: print("%-7s %-14s %-44r %-r" % (term, value, out, stack))
        if term == 'brackl':
            stack.append(out)
            out = []
        elif term == 'brackr':
            assert stack, "Trouble with nesting of brackets"
            tmpout, out = out, stack.pop(-1)
            out.append(tmpout)
        elif term == 'num':
            v = float(value)
            if v.is_integer(): v = int(v)
            out.append(v)
        elif term == 'sq':
            out.append(value[1:-1].replace(r'\"', '"'))
        elif term == 's':
            out.append(value)
        else:
            raise NotImplementedError("Error: %r" % (term, value))
    assert not stack, "Trouble with nesting of brackets"
    return out[0]


@dataclass
class KiCadPro:
    """The ``project`` token defines a KiCad layout according to the project file format used in
        ``.kicad_pro`` files.

        """
    netSetting: str = ""
    """The ``netSetting`` token defines the net and netClass"""

    board: str = ""
    """The ``boardSetting`` token defines the design rules"""

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

        object = self
        for i in range(len(exp)):
            item = exp[i]
            if item == '"net_settings":':
                object.netSetting = NetSetting().from_sexpr(exp[i+1])
            elif item == '"board":':
                object.board = BoardSetting().from_sexpr(exp[i+1])

        return object

    def from_file(self, filepath: str, encoding: Optional[str] = None):
        """Load a project directly from a KiCad project file (`.kicad_pro`) and sets the
        ``self.filePath`` attribute to the given file path.

        Args:
            - filepath (str): Path or path-like object that points to the file
            - encoding (str, optional): Encoding of the input file. Defaults to None (platform
                                        dependent encoding).

        Raises:
            - Exception: If the given path is not a file

        Returns:
            - NetSetting: Object of the Schematic class initialized with the given KiCad schematic
        """
        if not path.isfile(filepath):
            raise Exception("Given path is not a file!")

        with open(filepath, 'r', encoding=encoding) as infile:
            item = self.from_sexpr(parse_sexp(infile.read()))
            item.filePath = filepath
            return item


if __name__ == '__main__':
    kicad_pro = "bench4/bm4.unrouted.kicad_pro"
    project = KiCadPro().from_file(kicad_pro)
    print('kicad_pro')
