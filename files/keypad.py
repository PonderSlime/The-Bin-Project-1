class Keypad:
    def __init__(self, rows, columns):
        self.rows = rows
        self.columns = columns

    def init(self):
        for row in self.rows:
            row.init(mode=row.IN, pull=row.PULL_UP)

        for col in self.columns:
            col.init(mode=col.OUT, value=1)

    def get_key(self):
        for col_num, col in enumerate(self.columns):
            col.value(0)
            for row_num, row in enumerate(self.rows):
                if row.value() == 0:
                    while row.value() == 0:
                        pass  # Wait for key release
                    return self._key_for_position(row_num, col_num)
            col.value(1)
        return None

    def _key_for_position(self, row, col):
        # This is a simple mapping for a 4x4 keypad. Adjust as needed for your keypad layout.
        key_map = [
            ['1', '2', '3', 'A'],
            ['4', '5', '6', 'B'],
            ['7', '8', '9', 'C'],
            ['*', '0', '#', 'D']
        ]

        return key_map[row][col]
