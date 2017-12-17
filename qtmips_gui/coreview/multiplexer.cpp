#include "multiplexer.h"

using namespace coreview;

//////////////////////
#define WIDTH 20
#define HEIGHT 20
#define PENW 1
//////////////////////

Multiplexer::Multiplexer(unsigned size) {
    this->size = size;
    seton = 0;
    ctlfrom = false;
    con_ctl = new Connector();
    con_out = new Connector();
    con_in = new Connector*[size];
    for (unsigned i = 0; i < size; i++)
        con_in[i] = new Connector();
    setPos(x(), y()); // Set connectors possitions
}

Multiplexer::~Multiplexer() {
    delete con_ctl;
    delete con_out;
    for (unsigned i = 0; i < size; i++)
        delete con_in[i];
    delete con_in;
}

QRectF Multiplexer::boundingRect() const {
    return QRectF(-PENW / 2, -PENW / 2, WIDTH + PENW, (HEIGHT * size) + PENW);
}

void Multiplexer::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    const QPointF poly[] = {
        QPointF(0, 0),
        QPointF(WIDTH, WIDTH),
        QPointF(WIDTH, (HEIGHT * size) - WIDTH),
        QPointF(0, HEIGHT * size)
    };
    painter->drawPolygon(poly, sizeof(poly) / sizeof(QPointF));

    painter->setPen(QColor(200, 200, 200));
    painter->drawLine(0, (HEIGHT / 2) + (seton * HEIGHT), WIDTH, (HEIGHT * size) / 2);
}

void Multiplexer::setPos(qreal x, qreal y) {
    QGraphicsItem::setPos(x, y);
    if (ctlfrom)
        con_ctl->setPos(x + (WIDTH / 2), y + (WIDTH / 2));
    else
        con_ctl->setPos(x + (WIDTH / 2), y + (HEIGHT * size) - (WIDTH / 2));
    con_out->setPos(x + WIDTH, y + ((HEIGHT *size) / 2));
    for (unsigned i = 0; i < size; i++)
        con_in[i]->setPos(x, y + (HEIGHT / 2) + (i * HEIGHT));
}

const Connector *Multiplexer::connector_ctl() const {
    return con_ctl;
}

const Connector *Multiplexer::connector_out() const {
    return con_out;
}

const Connector *Multiplexer::connector_in(unsigned i) const {
    SANITY_ASSERT(i < size, "Multiplexer: requested out of range input connector");
    return con_in[i];
}

void Multiplexer::set(unsigned i) {
    seton = i;
    update(boundingRect());
}

void Multiplexer::setCtl(bool up) {
    ctlfrom = up;
    setPos(x(), y()); // Update connectors
}
