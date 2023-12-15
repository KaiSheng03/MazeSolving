#ifndef CELL_H
#define CELL_H

#include "Arduino.h"

class Cell{
  private:
    int row;
    int column;
    bool visited;
    bool useless;
    int callerRow;
    int callerColumn;

  public:
    Cell();
    Cell(int, int);
    int getRow() const;
    int getColumn() const;
    void setCoordinate(int, int);
    void setCaller(int, int);
    Cell getCaller() const;
    Cell operator-(Cell&);
    bool operator==(const Cell&);
    void markVisited();
    void markUseless();
    bool getVisited() const;
    bool getUseless() const;
    void printCoordinate() const;
};

#endif
