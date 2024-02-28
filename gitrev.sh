#!/bin/bash
# writes the GIT hash into the file gitrev.h
OUTFILE="uart_gitrev.h"
GITSHA=$(git rev-parse HEAD | head -c 8)
GITSHA_INT32=$(echo $(( 16#$GITSHA)))
# only update the file if the content has changed
NEWCONTENT="\n#ifndef _UART_GITREV_H\n#define _UART_GITREV_H\n\n#define UART_GIT_REV \"${GITSHA}\"\n#define UART_GIT_REV_INT ${GITSHA_INT32}\n\n#endif /* _UART_GITREV_H */\n"
grep "${GITSHA}" $OUTFILE > /dev/null
if [ $? -ne 0 ]; then
  printf "$NEWCONTENT" > $OUTFILE
  echo "file $OUTFILE updated"
else
  echo "file content unchanged"
fi

