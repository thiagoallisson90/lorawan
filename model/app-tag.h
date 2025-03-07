/*
 * Copyright (c) 2024 Federal University of Piauí
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Thiago Allisson <thiago.allisson@ufpi.edu.br>
 */

#ifndef APP_TAG_H
#define APP_TAG_H

#include "ns3/tag.h"

namespace ns3
{
namespace lorawan
{

enum MsgType
{
  NONE,
  RES_IMR,
  COMM_IMR,
  ODMR,
  BILLING_INFO,
  RCC,
  PCC,
  AN
};

/**
 * \ingroup lorawan
 *
 * Tag used to save various data about a packet, like its Spreading Factor and data about
 * interference.
 */
class AppTag : public Tag
{
  public:
    /**
     *  Register this type.
     *  \return The object TypeId.
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    AppTag(uint8_t msgType);
    AppTag();

    ~AppTag() override; //!< Destructor

    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    uint32_t GetSerializedSize() const override;
    void Print(std::ostream& os) const override;
     
    uint8_t GetMsgType() const;
    void SetMsgType(uint8_t msgType);

  private:
    uint8_t m_msgType;
};
} // namespace lorawan
} // namespace ns3
#endif
